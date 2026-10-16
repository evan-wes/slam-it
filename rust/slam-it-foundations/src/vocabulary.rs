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
