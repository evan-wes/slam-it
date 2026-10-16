"""Place recognition's vocabulary: a tree of binary words over ORB descriptors (DBoW2's).

A vocabulary quantises a descriptor to a word, so that two images compare as two histograms of
words rather than descriptor by descriptor (Gálvez-López and Tardós, "Bags of Binary Words for
Fast Place Recognition in Image Sequences", T-RO 2012; ORB-SLAM's place recognition):

- **The tree** is built by hierarchical k-means on descriptors from training images
  (:func:`train`): the root's descriptors split into ``k`` clusters, each cluster's into ``k``
  again, ``levels`` deep. Distances are Hamming distances; a cluster's centre is the bitwise
  majority of its descriptors (a tie sets the bit, as DBoW2's ``FORB::meanValue``); clusters are
  seeded by k-means++ from a fixed seed, and refined until no descriptor changes cluster (or
  :data:`MAX_KMEANS_ROUNDS`). A node with ``k`` descriptors or fewer has one child per
  descriptor. The leaves are the words.
- **A descriptor's word:** from the root, to the nearest child at each level (the lowest index
  among equals), to a leaf (:func:`quantize`). Its node at :data:`DIRECT_LEVEL` groups it with
  the features that matching by word compares it with
  (:func:`slam_it.foundations.matching.search_by_bow`; DBoW2's direct index; ORB-SLAM keeps the
  node 4 levels above the leaves of its 6, so level 2).
- **Weights:** a word's inverse document frequency, ``log(N / N_i)``, ``N`` the training images
  that yielded a descriptor and ``N_i`` those with the word (DBoW2 counts every image it is
  given: an image without a descriptor would only shift every weight by the same amount). An
  image's vector is each word's count times its weight, normalised to unit L1 norm (:func:`bow`);
  two vectors score ``1 - |v - w|_1 / 2`` (DBoW2's L1 score, :func:`score`): 1 for the same
  image, 0 for no word in common.

The arrays are :data:`ARRAYS`; a vocabulary's folder of them is read and written by
:mod:`slam_it.eval.vocabularies` (the algorithm rungs take a vocabulary, never its files).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations.arrays import as_count
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.orb import DESCRIPTOR_BYTES, distance

ARRAYS = ("descriptors", "parents", "children", "words", "weights")
"""A vocabulary's arrays (each a ``<name>.npy`` in its folder: :mod:`slam_it.eval.vocabularies`)."""

DIRECT_LEVEL = 2
"""The tree level whose nodes group a frame's features for matching (DBoW2's direct index)."""

MAX_K = 64
"""The most children a node may have."""

MAX_LEVELS = 8
"""The deepest a tree may be."""

MAX_KMEANS_ROUNDS = 100
"""The most assignment rounds one node's k-means runs (DBoW2 runs until none changes)."""

CHUNK = 1 << 18
"""Descriptors compared with a node's centres at once, bounding the memory a step takes."""


@dataclass(frozen=True)
class Vocabulary:
    """A tree of binary words (the module docstring).

    Attributes
    ----------
    k : int
        The most children a node has.
    levels : int
        The deepest a leaf lies (the root is level 0).
    descriptors : ndarray of uint8, shape (N, DESCRIPTOR_BYTES)
        Each node's centre.
    parents : ndarray of int32, shape (N,)
        Each node's parent; -1 for the root, node 0.
    children : ndarray of int32, shape (N, 2)
        Each node's first child and its count; a leaf's count is 0.
    words : ndarray of int32, shape (N,)
        Each leaf's word, numbered in node order; -1 for an inner node.
    weights : ndarray of float64, shape (W,)
        Each word's weight, its inverse document frequency (0 for a word no training image
        reached).
    info : dict
        What training records of its making: the training set, its images and descriptors, the
        seed (its folder's ``training.json``).
    """

    k: int
    levels: int
    descriptors: NDArray[np.uint8]
    parents: NDArray[np.int32]
    children: NDArray[np.int32]
    words: NDArray[np.int32]
    weights: NDArray[np.float64]
    info: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Check the tree's shape.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If an array has the wrong dtype or shape, the root is not node 0, a child does not
            name its parent, the children of a node are not consecutive, a node has more than
            ``k`` children, a leaf lies deeper than ``levels``, or the words are not the leaves
            numbered in node order with one weight each.
        """
        as_count(self.k, "k", 2, MAX_K)
        as_count(self.levels, "levels", 1, MAX_LEVELS)
        n = len(self.parents)
        expected = {
            "descriptors": (np.uint8, (n, DESCRIPTOR_BYTES)),
            "parents": (np.int32, (n,)),
            "children": (np.int32, (n, 2)),
            "words": (np.int32, (n,)),
        }
        for name, (dtype, shape) in expected.items():
            array = getattr(self, name)
            if not isinstance(array, np.ndarray) or array.dtype != dtype or array.shape != shape:
                raise InvalidInputError(f"{name}: expected {np.dtype(dtype)} {shape}")
        if self.weights.dtype != np.float64 or self.weights.ndim != 1:
            raise InvalidInputError("weights: expected float64 (W,)")
        if n == 0 or self.parents[0] != -1:
            raise InvalidInputError("parents: the root must be node 0, its parent -1")
        if np.any(self.parents[1:] < 0) or np.any(self.parents[1:] >= np.arange(1, n)):
            raise InvalidInputError("parents: each node's parent must come before it")
        first, count = self.children[:, 0].astype(np.int64), self.children[:, 1].astype(np.int64)
        if np.any(count < 0) or np.any(count > self.k):
            raise InvalidInputError(f"children: a node has from 0 to k ({self.k}) children")
        inner = np.flatnonzero(count > 0)
        if (
            count.sum() != n - 1
            or np.any(first[inner] < 1)
            or np.any(first[inner] + count[inner] > n)
        ):
            raise InvalidInputError("children: expected every node but the root once, in range")
        starts = np.cumsum(count[inner]) - count[inner]
        slots = np.repeat(first[inner], count[inner]) + (
            np.arange(n - 1) - np.repeat(starts, count[inner])
        )
        owners = np.repeat(inner, count[inner])
        if not (
            np.array_equal(np.sort(slots), np.arange(1, n))
            and np.array_equal(self.parents[slots], owners)
        ):
            raise InvalidInputError("children: a node's children must be consecutive and name it")
        if np.any(_depths(self.parents)[count == 0] > self.levels):
            raise InvalidInputError(f"levels: a leaf lies deeper than {self.levels}")
        leaves = np.flatnonzero(count == 0)
        if not (
            np.array_equal(self.words[leaves], np.arange(len(leaves)))
            and np.all(self.words[inner] == -1)
            and len(self.weights) == len(leaves)
        ):
            raise InvalidInputError(
                "words: expected the leaves numbered in node order, one weight each"
            )
        if not np.all(np.isfinite(self.weights) & (self.weights >= 0.0)):
            raise InvalidInputError("weights: expected non-negative finite numbers")

    @property
    def size(self) -> int:
        """The number of words."""
        return len(self.weights)


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.vocabulary`` submodule provides.

    A vocabulary crosses as a dict of :class:`Vocabulary`'s fields (:func:`fields`).
    """

    def quantize(
        self, vocabulary: dict[str, Any], descriptors: NDArray[np.uint8]
    ) -> dict[str, Any]:
        """Return ``words`` and ``nodes`` (int64)."""
        ...

    def bow(self, vocabulary: dict[str, Any], words: NDArray[np.int64]) -> dict[str, Any]:
        """Return ``ids`` (int64) and ``values`` (float64)."""
        ...

    def score(
        self,
        ids1: NDArray[np.int64],
        values1: NDArray[np.float64],
        ids2: NDArray[np.int64],
        values2: NDArray[np.float64],
    ) -> float:
        """Return the L1 score."""
        ...


def fields(vocabulary: Vocabulary) -> dict[str, Any]:
    """Return a vocabulary as it crosses to a compiled backend: ``k``, ``levels``, its arrays.

    Parameters
    ----------
    vocabulary : Vocabulary
        The vocabulary.

    Returns
    -------
    dict
        Its fields, the arrays shared, not copied.
    """
    return {"k": vocabulary.k, "levels": vocabulary.levels} | {
        name: getattr(vocabulary, name) for name in ARRAYS
    }


def _depths(parents: NDArray[np.int32]) -> NDArray[np.int64]:
    """Each node's level, the root's 0 (each parent before its children)."""
    depth = np.zeros(len(parents), dtype=np.int64)
    for _ in range(MAX_LEVELS + 1):  # a level a pass
        depth[1:] = depth[parents[1:]] + 1
    return depth


def _as_descriptors(descriptors: ArrayLike, name: str = "descriptors") -> NDArray[np.uint8]:
    """Check descriptors: uint8 rows of DESCRIPTOR_BYTES."""
    out = np.asarray(descriptors)
    if out.dtype != np.uint8 or out.ndim != 2 or out.shape[1] != DESCRIPTOR_BYTES:
        raise InvalidInputError(f"{name}: expected uint8 (n, {DESCRIPTOR_BYTES})")
    return out


def quantize(
    vocabulary: Vocabulary, descriptors: ArrayLike, *, backend: str = "python"
) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    """Return each descriptor's word, and its node at :data:`DIRECT_LEVEL`.

    Parameters
    ----------
    vocabulary : Vocabulary
        The vocabulary.
    descriptors : array_like of uint8, shape (n, DESCRIPTOR_BYTES)
        The descriptors.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    words : ndarray of int64, shape (n,)
        Each one's word: the leaf reached from the root, at each level to the nearest child by
        Hamming distance (the lowest index among equals).
    nodes : ndarray of int64, shape (n,)
        Its node at :data:`DIRECT_LEVEL`, or its leaf if that lies higher.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the descriptors are not uint8 rows of DESCRIPTOR_BYTES.
    """
    desc = _as_descriptors(descriptors)
    return backends.call(
        backend,
        __name__,
        "quantize",
        reference=lambda: _quantize(vocabulary, desc),
        arguments={"vocabulary": fields(vocabulary), "descriptors": np.ascontiguousarray(desc)},
        decode=lambda raw, _: (
            backends.checked_array(raw["words"], np.int64, 1, "quantize words"),
            backends.checked_array(raw["nodes"], np.int64, 1, "quantize nodes"),
        ),
    )


def _quantize(
    vocabulary: Vocabulary, desc: NDArray[np.uint8]
) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    """:func:`quantize`'s reference, on checked descriptors."""
    first = vocabulary.children[:, 0].astype(np.int64)
    count = vocabulary.children[:, 1].astype(np.int64)
    current = np.zeros(len(desc), dtype=np.int64)
    direct = current.copy()
    offsets = np.arange(vocabulary.k)
    for level in range(1, vocabulary.levels + 1):
        for lo in range(0, len(desc), CHUNK):
            at = current[lo : lo + CHUNK]
            moving = count[at] > 0
            if not np.any(moving):
                continue
            nodes = at[moving]
            candidates = first[nodes, None] + offsets[None, :]
            valid = offsets[None, :] < count[nodes, None]
            centres = vocabulary.descriptors[np.where(valid, candidates, 0)]
            distances = distance(desc[lo : lo + CHUNK][moving][:, None, :], centres)
            distances[~valid] = np.iinfo(np.int64).max
            chosen = candidates[np.arange(len(nodes)), np.argmin(distances, axis=1)]
            at[moving] = chosen
        if level == DIRECT_LEVEL:  # a leaf reached higher stays where it is
            direct = current.copy()
    if vocabulary.levels < DIRECT_LEVEL:
        direct = current.copy()
    return vocabulary.words[current].astype(np.int64), direct


def bow(
    vocabulary: Vocabulary, words: ArrayLike, *, backend: str = "python"
) -> tuple[NDArray[np.int64], NDArray[np.float64]]:
    """Return an image's word vector: each word's count times its weight, L1-normalised.

    Parameters
    ----------
    vocabulary : Vocabulary
        The vocabulary.
    words : array_like of int, shape (n,)
        The image's features' words (:func:`quantize`).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ids : ndarray of int64, shape (m,)
        The words with a weight above 0, ascending.
    values : ndarray of float64, shape (m,)
        Their values, summing to 1 (none if no word has a weight: an empty vector).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a word is not one of the vocabulary's.
    """
    found = np.asarray(words)
    if found.ndim != 1 or (found.size and not np.issubdtype(found.dtype, np.integer)):
        raise InvalidInputError("words: expected integers (n,)")
    if found.size and (found.min() < 0 or found.max() >= vocabulary.size):
        raise InvalidInputError(
            f"words: expected words of the vocabulary, 0 to {vocabulary.size - 1}"
        )
    checked = np.ascontiguousarray(found, dtype=np.int64)
    return backends.call(
        backend,
        __name__,
        "bow",
        reference=lambda: _bow(vocabulary, checked),
        arguments={"vocabulary": fields(vocabulary), "words": checked},
        decode=lambda raw, _: (
            backends.checked_array(raw["ids"], np.int64, 1, "bow ids"),
            backends.checked_array(raw["values"], np.float64, 1, "bow values"),
        ),
    )


def _bow(
    vocabulary: Vocabulary, words: NDArray[np.int64]
) -> tuple[NDArray[np.int64], NDArray[np.float64]]:
    """:func:`bow`'s reference, on checked words."""
    ids, counts = np.unique(words, return_counts=True)
    values = counts * vocabulary.weights[ids]
    kept = values > 0.0
    ids, values = ids[kept], values[kept]
    total = 0.0
    for value in values.tolist():  # in word order, as the compiled backends add them
        total += value
    return ids, values / total if total > 0.0 else values


def score(
    first: tuple[ArrayLike, ArrayLike],
    second: tuple[ArrayLike, ArrayLike],
    *,
    backend: str = "python",
) -> float:
    """Return two word vectors' L1 score, ``1 - |v - w|_1 / 2`` (DBoW2's ``L1Scoring``).

    Computed over the words the two share, ascending, as ``(|v| + |w| - |v - w|) / 2`` summed,
    so that the three languages add the same terms in the same order.

    Parameters
    ----------
    first, second : tuple of array_like
        Two vectors from :func:`bow`: their words (ascending) and values.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    float
        From 0 (no word shared) to 1 (the same vector).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a vector's words and values differ in length, or its words do not ascend.
    """
    (a_ids, a_values), (b_ids, b_values) = (as_vector(*first), as_vector(*second))
    return backends.call(
        backend,
        __name__,
        "score",
        reference=lambda: _score(a_ids, a_values, b_ids, b_values),
        arguments={"ids1": a_ids, "values1": a_values, "ids2": b_ids, "values2": b_values},
        decode=lambda raw, _: float(raw),
    )


def _score(
    a_ids: NDArray[np.int64],
    a_values: NDArray[np.float64],
    b_ids: NDArray[np.int64],
    b_values: NDArray[np.float64],
) -> float:
    """:func:`score`'s reference, on checked vectors."""
    _, ia, ib = np.intersect1d(a_ids, b_ids, assume_unique=True, return_indices=True)
    total = 0.0
    for v, w in zip(a_values[ia].tolist(), b_values[ib].tolist(), strict=True):
        total += abs(v) + abs(w) - abs(v - w)
    return total / 2.0


def as_vector(ids: ArrayLike, values: ArrayLike) -> tuple[NDArray[np.int64], NDArray[np.float64]]:
    """Check a word vector (:func:`bow`'s): its words ascending, one finite value each.

    Parameters
    ----------
    ids : array_like of int, shape (m,)
        The words.
    values : array_like of float, shape (m,)
        Their values.

    Returns
    -------
    ids : ndarray of int64, shape (m,)
        The words, contiguous.
    values : ndarray of float64, shape (m,)
        Their values, contiguous.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the words are not integers, or not one per value, or do not ascend, or a value is
        not finite.
    """
    i, v = np.asarray(ids), np.asarray(values, dtype=np.float64)
    if i.ndim != 1 or v.shape != i.shape or (i.size and not np.issubdtype(i.dtype, np.integer)):
        raise InvalidInputError("vector: expected (m,) words and (m,) values")
    if np.any(np.diff(i) <= 0) or not np.all(np.isfinite(v)):
        raise InvalidInputError("vector: expected ascending words and finite values")
    return np.ascontiguousarray(i, dtype=np.int64), np.ascontiguousarray(v)


def _majority(descriptors: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return the bitwise majority of descriptors, a tie setting the bit (DBoW2's ``meanValue``)."""
    ones = np.zeros(8 * DESCRIPTOR_BYTES, dtype=np.int64)
    for lo in range(0, len(descriptors), CHUNK):
        ones += np.unpackbits(descriptors[lo : lo + CHUNK], axis=1).sum(axis=0, dtype=np.int64)
    return np.packbits(2 * ones >= len(descriptors))


def _assign(descriptors: NDArray[np.uint8], centres: NDArray[np.uint8]) -> NDArray[np.int64]:
    """Each descriptor's nearest centre (the lowest index among equals)."""
    out = np.empty(len(descriptors), dtype=np.int64)
    for lo in range(0, len(descriptors), CHUNK):
        block = descriptors[lo : lo + CHUNK, None, :]
        out[lo : lo + CHUNK] = np.argmin(distance(block, centres[None, :, :]), axis=1)
    return out


def _seeds(descriptors: NDArray[np.uint8], k: int, rng: np.random.Generator) -> NDArray[np.uint8]:
    """k-means++ centres as DBoW2 picks them: each next with probability its distance."""
    centres = [descriptors[rng.integers(len(descriptors))]]
    nearest = distance(descriptors, centres[0])
    while len(centres) < k:
        total = float(nearest.sum())
        if total <= 0.0:  # every descriptor is a centre already
            break
        cut = 0.0
        while cut == 0.0:  # DBoW2 draws again on a zero cut
            cut = rng.uniform(0.0, total)
        index = min(int(np.searchsorted(np.cumsum(nearest), cut, side="left")), len(nearest) - 1)
        centres.append(descriptors[index])
        nearest = np.minimum(nearest, distance(descriptors, centres[-1]))
    return np.stack(centres)


def _kmeans(
    descriptors: NDArray[np.uint8], k: int, rng: np.random.Generator
) -> tuple[NDArray[np.uint8], NDArray[np.int64]]:
    """Cluster descriptors: centres, and each descriptor's cluster (empty clusters dropped)."""
    centres = _seeds(descriptors, k, rng)
    labels = _assign(descriptors, centres)
    for _ in range(MAX_KMEANS_ROUNDS):
        for c in range(len(centres)):  # an emptied cluster keeps its centre
            members = labels == c
            if np.any(members):
                centres[c] = _majority(descriptors[members])
        again = _assign(descriptors, centres)
        if np.array_equal(again, labels):
            break
        labels = again
    used = np.unique(labels)
    remap = np.full(len(centres), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return centres[used], remap[labels]


def train(  # noqa: PLR0913 -- the descriptors, their images, the tree's shape, the seed, a record
    descriptors: ArrayLike,
    images: ArrayLike,
    *,
    k: int = 10,
    levels: int = 6,
    seed: int = 0,
    info: dict[str, Any] | None = None,
) -> Vocabulary:
    """Build a vocabulary by hierarchical k-means (the module docstring).

    Parameters
    ----------
    descriptors : array_like of uint8, shape (n, DESCRIPTOR_BYTES)
        The training images' descriptors.
    images : array_like of int, shape (n,)
        The image each came from, for the words' document frequencies.
    k : int, optional
        A node's most children (10, DBoW2's and ORB-SLAM's).
    levels : int, optional
        The tree's depth (6, ORB-SLAM's; 5 gives a tenth of the words).
    seed : int, optional
        The k-means++ seeding's seed: each node draws from its own stream, ``(seed, node)``.
    info : dict, optional
        What to record of the training set (its folder's ``training.json``).

    Returns
    -------
    Vocabulary
        The tree, its nodes in breadth-first order, each node's children consecutive.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the descriptors are malformed or empty, `images` is not one non-negative integer per
        descriptor, or `k` or `levels` is out of range.
    """
    desc = _as_descriptors(descriptors)
    owner = np.asarray(images)
    if (
        owner.shape != (len(desc),)
        or not np.issubdtype(owner.dtype, np.integer)
        or np.any(owner < 0)
    ):
        raise InvalidInputError("images: expected one non-negative integer per descriptor")
    if len(desc) == 0:
        raise InvalidInputError("descriptors: expected at least one")
    k, levels = as_count(k, "k", 2, MAX_K), as_count(levels, "levels", 1, MAX_LEVELS)
    centres = [np.zeros(DESCRIPTOR_BYTES, dtype=np.uint8)]
    parents, children = [-1], [[0, 0]]
    queue = deque([(0, np.arange(len(desc)), 0)])  # (node, its descriptors, its level)
    while queue:  # breadth first: a node's children are made together, consecutive
        node, members, level = queue.popleft()
        if len(members) <= k:  # a child per descriptor (DBoW2)
            groups = [members[[i]] for i in range(len(members))]
            child_centres = desc[members]
        else:
            rng = np.random.default_rng([seed, node])
            child_centres, labels = _kmeans(desc[members], k, rng)
            groups = [members[labels == c] for c in range(len(child_centres))]
        children[node] = [len(parents), len(groups)]
        for centre, group in zip(child_centres, groups, strict=True):
            child = len(parents)
            centres.append(centre)
            parents.append(node)
            children.append([0, 0])
            if level + 1 < levels and len(group) > 1:
                queue.append((child, group, level + 1))
    count = np.array([c[1] for c in children])
    words = np.full(len(parents), -1, dtype=np.int32)
    leaves = np.flatnonzero(count == 0)
    words[leaves] = np.arange(len(leaves), dtype=np.int32)
    tree = Vocabulary(
        k, levels, np.stack(centres), np.array(parents, dtype=np.int32),
        np.array(children, dtype=np.int32), words, np.zeros(len(leaves)),
    )  # fmt: skip
    found, _ = quantize(tree, desc)
    pairs = np.unique(np.column_stack([owner.astype(np.int64), found]), axis=0)
    seen = np.bincount(pairs[:, 1], minlength=len(leaves))
    n_images = len(np.unique(owner))
    weights = np.zeros(len(leaves))
    weights[seen > 0] = np.log(n_images / seen[seen > 0])
    record = {"images": n_images, "descriptors": len(desc), "seed": seed, **(info or {})}
    return Vocabulary(
        k, levels, tree.descriptors, tree.parents, tree.children, words, weights, record
    )
