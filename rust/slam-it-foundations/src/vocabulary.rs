//! Place recognition's vocabulary: the Rust core's mirror of `slam_it.foundations.vocabulary`.
//!
//! A tree of binary words over ORB descriptors (`DBoW2`'s): a descriptor's word is the leaf
//! reached from the root by the nearest child at each level; a frame's vector is each word's
//! count times its weight, L1-normalised; two vectors score `DBoW2`'s L1 score. The Python
//! module's docstring defines each step. Training is Python's alone; this core checks a tree,
//! and quantises, weighs and scores as Python does, adding the same terms in the same order. It
//! takes a tree's arrays, never its files: `slam_it_eval::vocabularies` reads a vocabulary's
//! folder.

use std::collections::BTreeMap;

use crate::error::{ErrorKind, Result, check_range, fail};
use crate::orb::Descriptor;

/// The tree level whose nodes group a frame's features for matching (the direct index).
pub const DIRECT_LEVEL: usize = 2;
/// The most children a node may have.
pub const MAX_K: usize = 64;
/// The deepest a tree may be.
pub const MAX_LEVELS: usize = 8;

/// A tree of binary words (`slam_it.foundations.vocabulary.Vocabulary`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Vocabulary {
    /// The most children a node has.
    pub k: usize,
    /// The deepest a leaf lies (the root is level 0).
    pub levels: usize,
    /// Each node's centre.
    pub descriptors: Vec<Descriptor>,
    /// Each node's parent; -1 for the root, node 0.
    pub parents: Vec<i32>,
    /// Each node's first child and its count; a leaf's count is 0.
    pub children: Vec<[i32; 2]>,
    /// Each leaf's word, numbered in node order; -1 for an inner node.
    pub words: Vec<i32>,
    /// Each word's weight.
    pub weights: Vec<f64>,
}

/// Each descriptor's word and its node at [`DIRECT_LEVEL`] (or its leaf, if higher).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Quantized {
    /// The words.
    pub words: Vec<i64>,
    /// The direct index's nodes.
    pub nodes: Vec<i64>,
}

/// A frame's word vector: its weighted words, ascending, their values summing to 1.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BowVector {
    /// The words.
    pub ids: Vec<i64>,
    /// Their values.
    pub values: Vec<f64>,
}

/// Each node's level, the root's 0 (each parent before its children, as checked).
///
/// # Arguments
///
/// * `parents` - The parents.
///
/// # Returns
///
/// The levels.
fn depths(parents: &[i32]) -> Vec<usize> {
    let mut out = vec![0; parents.len()];
    for node in 1..parents.len() {
        out[node] = out[usize::try_from(parents[node]).unwrap_or(0)] + 1;
    }
    out
}

/// The Hamming distance between two descriptors.
///
/// # Arguments
///
/// * `a` - A descriptor.
/// * `b` - Another.
///
/// # Returns
///
/// The bits in which they differ.
fn hamming(a: &Descriptor, b: &Descriptor) -> u32 {
    a.iter().zip(b).map(|(x, y)| (x ^ y).count_ones()).sum()
}

/// Checks a tree as `Vocabulary.__post_init__` does.
///
/// # Arguments
///
/// * `vocabulary` - The tree.
///
/// # Errors
///
/// `InvalidInput` naming what is malformed.
pub fn check(vocabulary: &Vocabulary) -> Result<()> {
    check_range("k", vocabulary.k, 2, MAX_K)?;
    check_range("levels", vocabulary.levels, 1, MAX_LEVELS)?;
    let n = vocabulary.parents.len();
    if vocabulary.descriptors.len() != n
        || vocabulary.children.len() != n
        || vocabulary.words.len() != n
    {
        return fail(
            ErrorKind::InvalidInput,
            "vocabulary: expected one entry per node in each array",
        );
    }
    if n == 0 || vocabulary.parents[0] != -1 {
        return fail(
            ErrorKind::InvalidInput,
            "parents: the root must be node 0, its parent -1",
        );
    }
    if (1..n).any(|node| usize::try_from(vocabulary.parents[node]).map_or(true, |p| p >= node)) {
        return fail(
            ErrorKind::InvalidInput,
            "parents: each node's parent must come before it",
        );
    }
    let mut total = 0;
    for (node, &[first, count]) in vocabulary.children.iter().enumerate() {
        let Some(count) = usize::try_from(count).ok().filter(|c| *c <= vocabulary.k) else {
            return fail(
                ErrorKind::InvalidInput,
                "children: a node has from 0 to k children",
            );
        };
        if count == 0 {
            continue; // a leaf: its first child means nothing
        }
        let Some(first) = usize::try_from(first)
            .ok()
            .filter(|f| *f >= 1 && f + count <= n)
        else {
            return fail(
                ErrorKind::InvalidInput,
                "children: expected every node but the root once, in range",
            );
        };
        if (first..first + count).any(|c| usize::try_from(vocabulary.parents[c]).ok() != Some(node))
        {
            return fail(
                ErrorKind::InvalidInput,
                "children: a node's children must be consecutive and name it",
            );
        }
        total += count;
    }
    if total + 1 != n {
        return fail(
            ErrorKind::InvalidInput,
            "children: expected every node but the root once, in range",
        );
    }
    let level = depths(&vocabulary.parents);
    let mut next = 0;
    for (node, depth) in level.iter().enumerate() {
        let leaf = vocabulary.children[node][1] == 0;
        if leaf && *depth > vocabulary.levels {
            return fail(
                ErrorKind::InvalidInput,
                "levels: a leaf lies deeper than the tree's levels",
            );
        }
        if vocabulary.words[node] != if leaf { next } else { -1 } {
            return fail(
                ErrorKind::InvalidInput,
                "words: expected the leaves numbered in node order, one weight each",
            );
        }
        next += i32::from(leaf);
    }
    if usize::try_from(next).ok() != Some(vocabulary.weights.len()) {
        return fail(
            ErrorKind::InvalidInput,
            "words: expected the leaves numbered in node order, one weight each",
        );
    }
    if !vocabulary
        .weights
        .iter()
        .all(|w| w.is_finite() && *w >= 0.0)
    {
        return fail(
            ErrorKind::InvalidInput,
            "weights: expected non-negative finite numbers",
        );
    }
    Ok(())
}

/// Each descriptor's word, and its node at [`DIRECT_LEVEL`].
///
/// # Arguments
///
/// * `vocabulary` - The vocabulary, checked.
/// * `descriptors` - The descriptors.
///
/// # Returns
///
/// The words and nodes: at each level the nearest child by Hamming distance, the lowest index
/// among equals.
#[must_use]
pub fn quantize(vocabulary: &Vocabulary, descriptors: &[Descriptor]) -> Quantized {
    let mut out = Quantized {
        words: Vec::with_capacity(descriptors.len()),
        nodes: Vec::with_capacity(descriptors.len()),
    };
    for descriptor in descriptors {
        let mut node = 0_usize;
        let mut direct = 0_usize;
        for level in 1..=vocabulary.levels {
            let [first, count] = vocabulary.children[node];
            if count > 0 {
                let first = usize::try_from(first).unwrap_or(0);
                let count = usize::try_from(count).unwrap_or(0);
                let mut best = u32::MAX;
                for c in first..first + count {
                    let d = hamming(descriptor, &vocabulary.descriptors[c]);
                    if d < best {
                        // the first of equals, the lowest index
                        best = d;
                        node = c;
                    }
                }
            }
            if level == DIRECT_LEVEL {
                direct = node;
            }
        }
        if vocabulary.levels < DIRECT_LEVEL {
            direct = node;
        }
        out.words.push(i64::from(vocabulary.words[node]));
        out.nodes.push(i64::try_from(direct).unwrap_or(i64::MAX));
    }
    out
}

/// A frame's word vector: each word's count times its weight, summed in word order and
/// normalised; words of weight 0 left out.
///
/// # Arguments
///
/// * `vocabulary` - The vocabulary.
/// * `words` - The frame's features' words.
///
/// # Errors
///
/// `InvalidInput` for a word outside the vocabulary.
pub fn bow(vocabulary: &Vocabulary, words: &[i64]) -> Result<BowVector> {
    let size = vocabulary.weights.len();
    let mut counts: BTreeMap<usize, u32> = BTreeMap::new();
    for &word in words {
        match usize::try_from(word) {
            Ok(w) if w < size => *counts.entry(w).or_insert(0) += 1,
            _ => {
                return fail(
                    ErrorKind::InvalidInput,
                    format!(
                        "words: expected words of the vocabulary, 0 to {}",
                        size.saturating_sub(1)
                    ),
                );
            }
        }
    }
    let mut out = BowVector::default();
    for (word, count) in counts {
        let value = f64::from(count) * vocabulary.weights[word];
        if value > 0.0 {
            out.ids.push(i64::try_from(word).unwrap_or(i64::MAX));
            out.values.push(value);
        }
    }
    let mut total = 0.0;
    for value in &out.values {
        // in word order, as Python adds them
        total += value;
    }
    if total > 0.0 {
        for value in &mut out.values {
            *value /= total;
        }
    }
    Ok(out)
}

/// Checks a word vector: as many values as words, the words ascending, the values finite.
///
/// # Arguments
///
/// * `vector` - The vector.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check_vector(vector: &BowVector) -> Result<()> {
    if vector.ids.len() != vector.values.len() {
        return fail(
            ErrorKind::InvalidInput,
            "vector: expected (m,) words and (m,) values",
        );
    }
    if vector.ids.windows(2).any(|w| w[1] <= w[0]) || !vector.values.iter().all(|v| v.is_finite()) {
        return fail(
            ErrorKind::InvalidInput,
            "vector: expected ascending words and finite values",
        );
    }
    Ok(())
}

/// `DBoW2`'s L1 score of two vectors: `(|v| + |w| - |v - w|) / 2` summed over their shared
/// words in ascending order.
///
/// # Arguments
///
/// * `first` - A vector.
/// * `second` - Another.
///
/// # Errors
///
/// `InvalidInput` for words and values of different lengths, or words that do not ascend.
pub fn score(first: &BowVector, second: &BowVector) -> Result<f64> {
    check_vector(first)?;
    check_vector(second)?;
    let (mut i, mut j, mut total) = (0, 0, 0.0);
    while i < first.ids.len() && j < second.ids.len() {
        match first.ids[i].cmp(&second.ids[j]) {
            std::cmp::Ordering::Less => i += 1,
            std::cmp::Ordering::Greater => j += 1,
            std::cmp::Ordering::Equal => {
                let (v, w) = (first.values[i], second.values[j]);
                total += v.abs() + w.abs() - (v - w).abs();
                i += 1;
                j += 1;
            }
        }
    }
    Ok(total / 2.0)
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/vocabulary.json`,
    //! which the Python and C++ tests share (quantising, word vectors and scores, exactly), and
    //! the core's checks: a malformed tree, a word outside the vocabulary, a malformed vector.

    use super::*;
    use serde_json::Value;

    /// The shared fixture, read into the tests when they are built.
    const FIXTURE: &str = include_str!("../../../tests/fixtures/foundations/vocabulary.json");

    /// Parses the shared fixture.
    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("the fixture is valid JSON")
    }

    /// Reads an integer of a type.
    fn integer<T: TryFrom<i64>>(value: &Value) -> T
    where
        T::Error: std::fmt::Debug,
    {
        T::try_from(value.as_i64().expect("an integer")).expect("in range")
    }

    /// Reads a list of integers of a type.
    fn integers<T: TryFrom<i64>>(value: &Value) -> Vec<T>
    where
        T::Error: std::fmt::Debug,
    {
        value
            .as_array()
            .expect("a list")
            .iter()
            .map(integer)
            .collect()
    }

    /// Reads a list of floats.
    fn floats(value: &Value) -> Vec<f64> {
        let values = value.as_array().expect("a list");
        values
            .iter()
            .map(|x| x.as_f64().expect("a float"))
            .collect()
    }

    /// Reads a descriptor set.
    fn descriptors(value: &Value) -> Vec<Descriptor> {
        serde_json::from_value(value.clone()).expect("rows of 32 bytes")
    }

    /// Reads the fixture's tree.
    fn tree(value: &Value) -> Vocabulary {
        let children = value["children"].as_array().expect("a list");
        Vocabulary {
            k: integer(&value["k"]),
            levels: integer(&value["levels"]),
            descriptors: descriptors(&value["descriptors"]),
            parents: integers(&value["parents"]),
            children: children
                .iter()
                .map(|c| [integer(&c[0]), integer(&c[1])])
                .collect(),
            words: integers(&value["words"]),
            weights: floats(&value["weights"]),
        }
    }

    /// Quantising, the vectors and the scores match the fixture, exactly.
    #[test]
    fn quantizing_weighing_and_scoring_match_the_fixture() {
        let fixture = fixture();
        let v = tree(&fixture["tree"]);
        check(&v).unwrap();
        let q = &fixture["quantize"];
        let found = quantize(&v, &descriptors(&q["descriptors"]));
        assert_eq!(found.words, integers::<i64>(&q["words"]));
        assert_eq!(found.nodes, integers::<i64>(&q["nodes"]));
        let mut vectors = Vec::new();
        for c in fixture["bow"].as_array().unwrap() {
            let got = bow(&v, &integers::<i64>(&c["words"])).unwrap();
            assert_eq!(got.ids, integers::<i64>(&c["ids"]));
            assert_eq!(got.values, floats(&c["values"]));
            vectors.push(got);
        }
        for c in fixture["score"].as_array().unwrap() {
            let (i, j): (usize, usize) = (integer(&c["first"]), integer(&c["second"]));
            assert_eq!(
                score(&vectors[i], &vectors[j]).unwrap(),
                c["score"].as_f64().unwrap()
            );
        }
    }

    /// A malformed tree is refused: a parent after its child, children past the int32 range, a
    /// leaf too deep, words out of node order, a word without a weight; a leaf's first child is
    /// not read.
    #[test]
    fn a_malformed_tree_is_refused() {
        let v = tree(&fixture()["tree"]);
        let refused = |change: &dyn Fn(&mut Vocabulary)| {
            let mut bad = v.clone();
            change(&mut bad);
            check(&bad).unwrap_err().kind
        };
        assert_eq!(refused(&|b| b.parents[5] = -7), ErrorKind::InvalidInput);
        assert_eq!(
            refused(&|b| b.children[1] = [i32::MAX, 3]),
            ErrorKind::InvalidInput
        );
        assert_eq!(refused(&|b| b.levels = 2), ErrorKind::InvalidInput);
        assert_eq!(refused(&|b| b.words.swap(3, 8)), ErrorKind::InvalidInput);
        assert_eq!(
            refused(&|b| {
                b.weights.pop();
            }),
            ErrorKind::InvalidInput
        );
        let mut leaf = v.clone();
        leaf.children[3] = [2_000_000_000, 0]; // node 3 is a leaf
        assert!(check(&leaf).is_ok());
    }

    /// A word past the last and a malformed vector are refused.
    #[test]
    fn a_word_outside_and_a_malformed_vector_are_refused() {
        let v = tree(&fixture()["tree"]);
        let past = i64::try_from(v.weights.len()).unwrap();
        assert_eq!(bow(&v, &[past]).unwrap_err().kind, ErrorKind::InvalidInput);
        let fine = BowVector {
            ids: vec![1, 4],
            values: vec![0.5, 0.5],
        };
        for bad in [
            BowVector {
                ids: vec![4, 1],
                values: vec![0.5, 0.5],
            },
            BowVector {
                ids: vec![1, 4],
                values: vec![1.0],
            },
            BowVector {
                ids: vec![1],
                values: vec![f64::INFINITY],
            },
        ] {
            assert_eq!(
                score(&fine, &bad).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
            assert_eq!(
                score(&bad, &fine).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
    }
}
