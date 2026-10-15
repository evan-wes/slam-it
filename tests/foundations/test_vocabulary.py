"""Place recognition's vocabulary: training, quantising, word vectors, scores and its files."""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.foundations import orb, vocabulary
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function

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


def test_a_vocabulary_round_trips_through_its_folder(tmp_path: Path) -> None:
    """Every array and the record survive, checked by their SHA-256."""
    descriptors, _, images = _clusters()
    vocab = vocabulary.train(descriptors, images, k=4, levels=2, seed=0, info={"set": "test"})
    vocabulary.write(tmp_path, vocab)
    back = vocabulary.read(tmp_path)
    for name in vocabulary.ARRAYS:
        np.testing.assert_array_equal(getattr(back, name), getattr(vocab, name))
    assert (back.k, back.levels, back.info) == (4, 2, vocab.info)
    record = json.loads((tmp_path / "vocabulary.json").read_text())
    assert (record["descriptor_version"], record["words"]) == (orb.DESCRIPTOR_VERSION, vocab.size)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ("tamper", "SHA-256 is not the recorded one"),
        ("descriptor", "trained on descriptor version 0"),
        ("format", "expected a slam-it-vocabulary of version 1"),
    ],
)
def test_reading_refuses_a_changed_or_foreign_vocabulary(
    tmp_path: Path, change: str, message: str
) -> None:
    """An array that changed, another descriptor's vocabulary, another format: refused."""
    descriptors, _, images = _clusters()
    vocabulary.write(tmp_path, vocabulary.train(descriptors, images, k=4, levels=2))
    record = json.loads((tmp_path / "vocabulary.json").read_text())
    if change == "tamper":
        weights = np.load(tmp_path / "weights.npy")
        np.save(tmp_path / "weights.npy", weights + 1.0)
    elif change == "descriptor":
        record["descriptor_version"] = 0
    else:
        record["version"] = 0
    (tmp_path / "vocabulary.json").write_text(json.dumps(record))
    with pytest.raises(InvalidInputError, match=message):
        vocabulary.read(tmp_path)


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


def test_the_default_vocabulary_reads_and_is_the_documented_one() -> None:
    """The committed vocabulary reads, its checksums hold, and it is 5 levels of k = 10."""
    vocab = vocabulary.read(vocabulary.DEFAULT)
    assert (vocab.k, vocab.levels) == (10, 5)
    assert 90_000 < vocab.size <= 100_000
    assert vocab.info["name"] == "arkitscenes-training"


@pytest.fixture(scope="module")
def default() -> vocabulary.Vocabulary:
    """Read the committed vocabulary."""
    return vocabulary.read(vocabulary.DEFAULT)


def _room_descriptors() -> NDArray[np.uint8]:
    """Two frames' ORB descriptors from the synthetic room."""
    from tests.edge import room  # noqa: PLC0415 -- the room renders only for these tests
    from tests.edge.support import FRAMES, OPTIONS  # noqa: PLC0415

    rendered = room.sequence(FRAMES)
    return np.concatenate(
        [orb.extract(rendered[i].gray, OPTIONS.extraction()).descriptors for i in (0, 20)]
    )


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_every_backend_reads_the_default_as_python_does(
    backend: str, default: vocabulary.Vocabulary
) -> None:
    """The compiled readers read the same five arrays, and find k and the levels in the tree."""
    if backend == "python":
        return
    reader = backends.implementation(
        require_function(backend, vocabulary.__name__, "read"), vocabulary.__name__
    )
    got = reader.read(str(vocabulary.DEFAULT))
    assert (got["k"], got["levels"]) == (default.k, default.levels)
    for name in vocabulary.ARRAYS:
        np.testing.assert_array_equal(got[name], getattr(default, name))


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
