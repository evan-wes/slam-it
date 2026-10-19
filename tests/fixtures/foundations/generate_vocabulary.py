"""Generate ``vocabulary.json``, the vocabulary's parity fixture: quantising, word vectors, scores.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_vocabulary

Independent of the module under test, nothing imported from slam_it: a tree built here (three
levels of k = 3, one node of two children, one leaf on level 1, a word of weight 0) with
seeded centres, and descriptors near them, some equally near two children; a descriptor's word
is a brute-force descent with Python integers' bit counts (the lowest index among equals), its
direct node the one it passes at level 2 (or its leaf, if higher); a vector is each word's count
times its weight, summed in word order and divided by the sum, a word of weight 0 left out; a
score sums ``|v| + |w| - |v - w|`` over the shared words in ascending order and halves it. The
three languages compute each float with the same IEEE operations in the same order, so the
policy is ``exact``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from tests.fixtures.foundations._layout import dumps

OUT = Path(__file__).with_name("vocabulary.json")
EXACT = {
    "comparison": "exact",
    "rtol": 0.0,
    "atol": 0.0,
    "reason": "integer descent; each float the same IEEE operations in the same order",
}
BYTES = 32
DIRECT_LEVEL = 2

Descriptor = list[int]


def _distance(a: Descriptor, b: Descriptor) -> int:
    """Count the differing bits of two descriptors."""
    return sum((x ^ y).bit_count() for x, y in zip(a, b, strict=True))


COUNTS = [3, 3, 2, 0, 3, 3, 3, 3, 0] + [0] * 12
"""Each node's children, breadth first: the root's three on level 1, of which node 1 has three,
node 2 two and node 3 none (a leaf on level 1); on level 2, nodes 4 to 7 have three each (leaves
on level 3) and node 8 none (a leaf on level 2)."""


def _tree(rng: np.random.Generator) -> dict[str, Any]:
    """Return the tree of :data:`COUNTS`, each node's children consecutive, seeded centres."""
    parents, children = [-1], []
    for node, count in enumerate(COUNTS):
        children.append([len(parents) if count else 0, count])
        parents += [node] * count
    assert len(parents) == len(COUNTS)
    centres = [[0] * BYTES] + [
        rng.integers(0, 256, BYTES).tolist() for _ in range(len(parents) - 1)
    ]
    leaves = [n for n, (_, c) in enumerate(children) if c == 0]
    words = [-1] * len(parents)
    for w, n in enumerate(leaves):
        words[n] = w
    weights = [round(float(x), 6) for x in rng.uniform(0.5, 4.0, len(leaves))]
    weights[3] = 0.0  # a word of weight 0: left out of every vector
    return {
        "k": 3,
        "levels": 3,
        "descriptors": centres,
        "parents": parents,
        "children": children,
        "words": words,
        "weights": weights,
    }


def _descend(tree: dict[str, Any], d: Descriptor) -> tuple[int, int, set[int]]:
    """Return a descriptor's word, its direct node, and the levels where two children tied."""
    node, direct, tied = 0, 0, set()
    for level in range(1, tree["levels"] + 1):
        first, count = tree["children"][node]
        if count:
            gaps = [_distance(d, tree["descriptors"][c]) for c in range(first, first + count)]
            if gaps.count(min(gaps)) > 1:
                tied.add(level)
            node = first + gaps.index(min(gaps))  # the lowest index among equals
        if level == DIRECT_LEVEL:
            direct = node
    return tree["words"][node], direct, tied


def _flipped(d: Descriptor, bits: list[int]) -> Descriptor:
    """Return a descriptor with some bits flipped (bit ``8 b + i`` is byte b's bit i)."""
    out = list(d)
    for bit in bits:
        out[bit // 8] ^= 1 << (bit % 8)
    return out


def _between(a: Descriptor, b: Descriptor) -> Descriptor:
    """Return a descriptor as near to `a` as to `b`: `a` with half their differing bits flipped."""
    differing = [8 * k + i for k in range(BYTES) for i in range(8) if (a[k] ^ b[k]) >> i & 1]
    return _flipped(a, differing[: len(differing) // 2])


def _descriptors(tree: dict[str, Any], rng: np.random.Generator) -> list[Descriptor]:
    """Return descriptors to quantise: near each node's centre, and ties between siblings."""
    out = []
    for centre in tree["descriptors"][1:]:
        flips = rng.choice(8 * BYTES, int(rng.integers(0, 12)), replace=False).tolist()
        out.append(_flipped(centre, flips))
    for first, count in tree["children"]:
        if count >= 2:  # a tie between the first two children: the lower index wins
            out.append(_between(tree["descriptors"][first], tree["descriptors"][first + 1]))
    out += [rng.integers(0, 256, BYTES).tolist() for _ in range(8)]
    return out


def _bow(tree: dict[str, Any], words: list[int]) -> tuple[list[int], list[float]]:
    """Return a frame's word vector: counts times weights, normalised, weight-0 words left out."""
    counts: dict[int, int] = {}
    for w in words:
        counts[w] = counts.get(w, 0) + 1
    ids, values = [], []
    for w in sorted(counts):
        value = counts[w] * tree["weights"][w]
        if value > 0.0:
            ids.append(w)
            values.append(value)
    total = 0.0
    for value in values:
        total += value
    return ids, [v / total for v in values] if total > 0.0 else values


def _score(first: tuple[list[int], list[float]], second: tuple[list[int], list[float]]) -> float:
    """Return two vectors' L1 score over their shared words, ascending."""
    shared = sorted(set(first[0]) & set(second[0]))
    total = 0.0
    for w in shared:
        v, u = first[1][first[0].index(w)], second[1][second[0].index(w)]
        total += abs(v) + abs(u) - abs(v - u)
    return total / 2.0


def main() -> None:
    """Write the fixture."""
    rng = np.random.default_rng(20261016)
    tree = _tree(rng)
    descriptors = _descriptors(tree, rng)
    found = [_descend(tree, d) for d in descriptors]
    tied = set().union(*(t for _, _, t in found))
    assert tied == {1, 2, 3}, f"ties on levels {sorted(tied)} only: the lowest-index rule untested"
    words = [w for w, _, _ in found]
    frames = [words[:12], words[12:], words[::2], [3, 3, 3], [], words[:12] * 2]
    vectors = [_bow(tree, f) for f in frames]
    pairs = [(i, j) for i in range(len(vectors)) for j in range(i, len(vectors))]
    doc = {
        "description": "The vocabulary's quantising, word vectors and scores, on a tree built "
        "here (generate_vocabulary.py).",
        "oracle": "a brute-force descent with Python integers' bit counts; vectors and scores "
        "summed in word order",
        "tolerance": EXACT,
        "tree": tree,
        "quantize": {"descriptors": descriptors, "words": words, "nodes": [n for _, n, _ in found]},
        "bow": [{"words": f, "ids": v[0], "values": v[1]} for f, v in zip(frames, vectors,
                                                                       strict=True)],
        "score": [{"first": i, "second": j, "score": _score(vectors[i], vectors[j])}
                  for i, j in pairs],
    }  # fmt: skip
    OUT.write_text(dumps(doc))
    print(
        f"wrote {OUT}: {len(descriptors)} descriptors, {len(frames)} vectors, {len(pairs)} scores"
    )


if __name__ == "__main__":
    main()
