"""Place recognition's vocabulary: training, quantising, word vectors and scores.

Its folder's tests are ``tests/eval/test_vocabularies.py``.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.eval import vocabularies
from slam_it.foundations import orb, vocabulary
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import assert_matches_fixture, load_fixture, require_function

DESCRIPTOR_SHA256 = "9d83b625eaed2a0f0a5690805875536339cf2e5e5ec6167325c1e41382a4ae4b"
"""The SHA-256 of the constants that define an ORB descriptor, as of DESCRIPTOR_VERSION 1."""


def _clusters(
    prototypes: int = 6, per: int = 40, flips: int = 8, seed: int = 0
) -> tuple[NDArray[np.uint8], NDArray[np.int64], NDArray[np.int64]]:
    """Descriptors around random prototypes, a few bits flipped each, and two images each."""
    rng = np.random.default_rng(seed)
    centres = rng.integers(0, 256, size=(prototypes, orb.DESCRIPTOR_BYTES), dtype=np.uint8)
    descriptors, labels = [], []
    for c, centre in enumerate(centres):
        bits = np.tile(np.unpackbits(centre), (per, 1))
        for row in bits:
            row[rng.choice(len(row), flips, replace=False)] ^= 1
        descriptors.append(np.packbits(bits, axis=1))
        labels += [c] * per
    images = np.arange(prototypes * per) % 12  # twelve images, every prototype in each
    return np.concatenate(descriptors), np.array(labels), images


def test_the_descriptor_version_stands_for_its_constants() -> None:
    """Changing what a descriptor holds must raise DESCRIPTOR_VERSION, and this pin with it.

    A vocabulary trained on other descriptors is refused by the version alone, so a change to
    the patch, smoothing, pattern or rotation that left the version would go unnoticed.
    """
    names = (
        "DESCRIPTOR_BYTES", "HALF_PATCH_SIZE", "UMAX", "ANGLE_BINS", "ROTATION_BITS",
        "ROTATIONS", "BLUR_KERNEL", "PATTERN",
    )  # fmt: skip
    parts = {name: getattr(orb, name) for name in names}
    digest = hashlib.sha256(json.dumps(parts, sort_keys=True).encode()).hexdigest()
    assert (orb.DESCRIPTOR_VERSION, digest) == (1, DESCRIPTOR_SHA256)


def test_training_never_mixes_two_clusters_under_a_node() -> None:
    """With more branches than clusters, no first-level node holds two clusters' descriptors.

    (k-means++ may seed two centres in one cluster and split it; it never joins two clusters
    when it has a branch for each.)
    """
    descriptors, labels, images = _clusters()
    vocab = vocabulary.train(descriptors, images, k=8, levels=2, seed=0)
    words, direct = vocabulary.quantize(vocab, descriptors)
    leaf = np.flatnonzero(vocab.words >= 0)[words]
    parent = vocab.parents[leaf]  # a first-level node: a leaf is at most two levels down
    for node in np.unique(parent):
        assert len(np.unique(labels[parent == node])) == 1
    assert vocab.info == {"images": 12, "descriptors": len(descriptors), "seed": 0}
    np.testing.assert_array_equal(direct, leaf)  # two levels: the direct index is the leaf


def test_training_is_deterministic_for_a_seed() -> None:
    """The same descriptors and seed give the same tree, array for array."""
    descriptors, _, images = _clusters()
    a = vocabulary.train(descriptors, images, k=4, levels=3, seed=5)
    b = vocabulary.train(descriptors, images, k=4, levels=3, seed=5)
    for name in vocabulary.ARRAYS:
        np.testing.assert_array_equal(getattr(a, name), getattr(b, name))


def test_a_node_of_k_descriptors_or_fewer_has_a_child_each() -> None:
    """DBoW2's trivial case: no k-means, a leaf per descriptor."""
    descriptors, _, _ = _clusters(prototypes=1, per=3)
    vocab = vocabulary.train(descriptors, np.arange(3), k=4, levels=3)
    assert vocab.children[0].tolist() == [1, 3]
    np.testing.assert_array_equal(vocab.descriptors[1:], descriptors)
    assert vocab.size == 3


def _byte(value: int) -> NDArray[np.uint8]:
    """Return a descriptor whose first byte is `value`, the rest zero."""
    out = np.zeros(orb.DESCRIPTOR_BYTES, dtype=np.uint8)
    out[0] = value
    return out


def _tree() -> vocabulary.Vocabulary:
    """Return a tree of three levels, k = 2, its centres in the first byte.

    Root 0; level 1: nodes 1 (0x00) and 2 (0xF0); level 2: 3 (0x00) and 4 (0x03) under 1, 5
    (0xF0) and 6 (0xFF) under 2; level 3: 7 (0xFF) and 8 (0xFE) under 6.
    """
    centres = [0x00, 0x00, 0xF0, 0x00, 0x03, 0xF0, 0xFF, 0xFF, 0xFE]
    return vocabulary.Vocabulary(
        k=2,
        levels=3,
        descriptors=np.stack([_byte(c) for c in centres]),
        parents=np.array([-1, 0, 0, 1, 1, 2, 2, 6, 6], dtype=np.int32),
        children=np.array(
            [[1, 2], [3, 2], [5, 2], [0, 0], [0, 0], [0, 0], [7, 2], [0, 0], [0, 0]],
            dtype=np.int32,
        ),
        words=np.array([-1, -1, -1, 0, 1, 2, -1, 3, 4], dtype=np.int32),
        weights=np.ones(5),
    )


def test_quantizing_descends_to_the_nearest_child_at_each_level() -> None:
    """The nearest child by Hamming distance at each level; ties to the lowest index.

    0x01 is one bit from 0x00 and from 0x03 at level 2: the tie goes to node 3. 0xF1 descends
    to 2, then 5. 0xFD, under 6, is one bit from 0xFF and two from 0xFE: word 3. A word's
    node at level 2 is kept as its direct index; a leaf at level 2 is its own.
    """
    vocab = _tree()
    words, direct = vocabulary.quantize(
        vocab, np.stack([_byte(v) for v in (0x01, 0xF1, 0xFD, 0x07)])
    )
    np.testing.assert_array_equal(words, [0, 2, 3, 1])
    np.testing.assert_array_equal(direct, [3, 5, 6, 4])


def test_a_word_in_every_image_weighs_nothing() -> None:
    """A word's weight is log(N / N_i): 0 when every training image has it."""
    descriptors, _, _ = _clusters(prototypes=2, per=10)
    everywhere = np.arange(20) % 2  # both images hold every prototype
    vocab = vocabulary.train(descriptors, everywhere, k=2, levels=1, seed=0)
    assert vocab.size == 2
    np.testing.assert_array_equal(vocab.weights, [0.0, 0.0])
    ids, values = vocabulary.bow(vocab, [0, 1, 1])
    assert (len(ids), len(values)) == (0, 0)  # no weighted word: an empty vector


def test_a_word_vector_is_weighted_counts_summing_to_one() -> None:
    """Each word's count times its weight, divided by their sum."""
    descriptors, labels, _ = _clusters(prototypes=3, per=6)
    vocab = vocabulary.train(descriptors, labels, k=3, levels=1, seed=0)  # an image a prototype
    ids, values = vocabulary.bow(vocab, [2, 0, 2, 2])
    np.testing.assert_array_equal(ids, [0, 2])
    raw = np.array([1.0, 3.0]) * vocab.weights[[0, 2]]
    np.testing.assert_allclose(values, raw / raw.sum(), rtol=0, atol=1e-15)
    assert sum(values.tolist()) == pytest.approx(1.0)


def test_the_l1_score_is_one_for_the_same_vector_and_zero_for_none_shared() -> None:
    """DBoW2's L1 score, symmetric, over the shared words."""
    v = (np.array([1, 4, 7]), np.array([0.5, 0.25, 0.25]))
    w = (np.array([4, 7, 9]), np.array([0.5, 0.25, 0.25]))
    assert vocabulary.score(v, v) == 1.0
    assert vocabulary.score(v, (np.array([2, 3]), np.array([0.5, 0.5]))) == 0.0
    assert vocabulary.score(v, w) == vocabulary.score(w, v) == 0.5  # min(.25,.5) + min(.25,.25)
    assert vocabulary.score(v, (np.zeros(0, dtype=np.int64), np.zeros(0))) == 0.0


def test_a_malformed_tree_is_refused() -> None:
    """Children not naming their parent, a leaf too deep, words out of node order."""
    descriptors, _, images = _clusters()
    vocab = vocabulary.train(descriptors, images, k=4, levels=2)
    parents = vocab.parents.copy()
    parents[-1] = 0
    with pytest.raises(InvalidInputError, match="children"):
        replace(vocab, parents=parents)
    with pytest.raises(InvalidInputError, match="deeper than 1"):
        replace(vocab, levels=1)
    words = vocab.words.copy()
    leaves = np.flatnonzero(words >= 0)
    words[leaves[:2]] = words[leaves[:2]][::-1]
    with pytest.raises(InvalidInputError, match="words"):
        replace(vocab, words=words)
    with pytest.raises(InvalidInputError, match="descriptors"):
        vocabulary.quantize(vocab, descriptors.astype(np.int16))


@pytest.fixture(scope="module")
def default() -> vocabulary.Vocabulary:
    """Return the committed vocabulary."""
    return vocabularies.default()


def _room_descriptors() -> NDArray[np.uint8]:
    """Two frames' ORB descriptors from the synthetic room."""
    from tests.edge import room  # noqa: PLC0415 -- the room renders only for these tests
    from tests.edge.support import FRAMES, OPTIONS  # noqa: PLC0415

    rendered = room.sequence(FRAMES)
    return np.concatenate(
        [orb.extract(rendered[i].gray, OPTIONS.extraction()).descriptors for i in (0, 20)]
    )


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_every_backend_quantizes_weighs_and_scores_as_python_does(
    backend: str, default: vocabulary.Vocabulary
) -> None:
    """Words, nodes, vectors and scores agree exactly: the same terms in the same order."""
    chosen = require_function(backend, vocabulary.__name__, "quantize")
    descriptors = _room_descriptors()
    words, nodes = vocabulary.quantize(default, descriptors)
    got_words, got_nodes = vocabulary.quantize(default, descriptors, backend=chosen)
    np.testing.assert_array_equal(got_words, words)
    np.testing.assert_array_equal(got_nodes, nodes)
    half = len(descriptors) // 2
    first = vocabulary.bow(default, words[:half])
    second = vocabulary.bow(default, words[half:])
    for expected, got in zip(
        (first, second),
        (vocabulary.bow(default, words[:half], backend=chosen),
         vocabulary.bow(default, words[half:], backend=chosen)),
        strict=True,
    ):  # fmt: skip
        np.testing.assert_array_equal(got[0], expected[0])
        np.testing.assert_array_equal(got[1], expected[1])
    expected_score = vocabulary.score(first, second)
    assert 0.0 < expected_score < 1.0
    assert vocabulary.score(first, second, backend=chosen) == expected_score
    assert vocabulary.score(first, first, backend=chosen) == vocabulary.score(first, first)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_every_backend_refuses_a_word_outside_the_vocabulary(
    backend: str, default: vocabulary.Vocabulary
) -> None:
    """A word past the last: InvalidInputError."""
    chosen = require_function(backend, vocabulary.__name__, "bow")
    with pytest.raises(InvalidInputError, match="words of the vocabulary"):
        vocabulary.bow(default, [default.size], backend=chosen)


VOCABULARY = load_fixture("foundations/vocabulary.json")
"""The parity fixture: quantising, word vectors and scores on a tree built by its generator."""

TRAINED_SHA256 = {
    "descriptors": "ed79984d99983e61508d5b10e5fec484a3a2c790789d5a0bdb5f341a918cc445",
    "parents": "30f5d888242099062ef3bfd3913221ce9d91c14fc3a87b71d40c7b48c7177b3b",
    "children": "c35ddabda51712c3af97f048bcfc74fac16ca03d2e8a45b2edd9bd48f5b08d1c",
    "words": "f84bce7368537a38df486ed4815f735213c341df84070659d731d12eba1988f6",
}
"""The SHA-256 of a tiny training's tree (:func:`_clusters`, k = 4, three levels, seed 5): its
four arrays of integers and bytes, which integer arithmetic and numpy's ``Generator`` decide."""


def _fixture_tree() -> vocabulary.Vocabulary:
    """Return the parity fixture's tree."""
    tree = VOCABULARY["tree"]
    return vocabulary.Vocabulary(
        tree["k"], tree["levels"], np.array(tree["descriptors"], dtype=np.uint8),
        np.array(tree["parents"], dtype=np.int32), np.array(tree["children"], dtype=np.int32),
        np.array(tree["words"], dtype=np.int32), np.array(tree["weights"]),
    )  # fmt: skip


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_every_backend_matches_the_parity_fixture(backend: str) -> None:
    """Words, direct nodes, vectors and scores are the fixture's brute force's, exactly."""
    chosen = require_function(backend, vocabulary.__name__, "quantize")
    tree, policy = _fixture_tree(), VOCABULARY["tolerance"]
    q = VOCABULARY["quantize"]
    words, nodes = vocabulary.quantize(
        tree, np.array(q["descriptors"], dtype=np.uint8), backend=chosen
    )
    assert_matches_fixture(words, q["words"], policy)
    assert_matches_fixture(nodes, q["nodes"], policy)
    vectors = []
    for case in VOCABULARY["bow"]:
        ids, values = vocabulary.bow(tree, case["words"], backend=chosen)
        assert_matches_fixture(ids, case["ids"], policy)
        assert_matches_fixture(values, case["values"], policy)
        vectors.append((ids, values))
    for case in VOCABULARY["score"]:
        got = vocabulary.score(vectors[case["first"]], vectors[case["second"]], backend=chosen)
        assert_matches_fixture(got, case["score"], policy)


def test_a_tied_bit_is_set_in_a_centre() -> None:
    """DBoW2's ``meanValue``: a bit set in half the descriptors is set; in fewer, not."""
    zero, ones = (
        np.zeros(orb.DESCRIPTOR_BYTES, np.uint8),
        np.full(orb.DESCRIPTOR_BYTES, 255, np.uint8),
    )
    np.testing.assert_array_equal(vocabulary._majority(np.stack([zero, ones])), ones)
    np.testing.assert_array_equal(vocabulary._majority(np.stack([zero, zero, ones])), zero)
    np.testing.assert_array_equal(vocabulary._majority(np.stack([zero, ones, ones])), ones)


def test_a_words_weight_is_the_log_of_the_images_over_those_with_it() -> None:
    """log(N / N_i): a word in 1 of 3 images weighs log 3, in 2 of 3 log 1.5, in none 0.

    Three descriptors, fewer than k: a leaf each (DBoW2's trivial case). The first two are the
    same, in images 0 and 1: both descend to the first leaf (the lowest index among equals), so
    it is in two images and the second in none.
    """
    descriptors, _, _ = _clusters(prototypes=2, per=1)
    three = np.stack([descriptors[0], descriptors[0], descriptors[1]])
    vocab = vocabulary.train(three, np.array([0, 1, 2]), k=4, levels=1)
    np.testing.assert_allclose(vocab.weights, [np.log(1.5), 0.0, np.log(3.0)], rtol=0, atol=1e-15)
    assert vocab.info["images"] == 3


def test_a_tiny_training_is_pinned_to_its_checksums() -> None:
    """A tiny training's tree, byte for byte, and its weights by their rule.

    The two 6-level vocabularies kept outside the repository are rebuilt "exactly", to the
    SHA-256 in ``vocabularies/local.json``, from about 15 GB of images. Their checksums depend on
    the seeding, the majority, the k-means rounds and numpy's ``Generator`` streams (which NEP
    19 does not hold across numpy's versions); this catches a change to any of them in a second,
    where a rebuild takes hours. The weights are ``np.log``'s, whose last bit may differ between
    CPUs, so they are checked against their rule rather than pinned.
    """
    descriptors, _, images = _clusters()
    vocab = vocabulary.train(descriptors, images, k=4, levels=3, seed=5)
    pinned = {name: getattr(vocab, name) for name in TRAINED_SHA256}
    got = {name: hashlib.sha256(array.tobytes()).hexdigest() for name, array in pinned.items()}
    assert got == TRAINED_SHA256
    words, _ = vocabulary.quantize(vocab, descriptors)
    holders = [len(np.unique(images[words == w])) for w in range(vocab.size)]
    expected = [np.log(12 / n) if n else 0.0 for n in holders]
    np.testing.assert_allclose(vocab.weights, expected, rtol=1e-15, atol=0)
