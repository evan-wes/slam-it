"""Generate ``matching.json``, the descriptor-matching parity fixture.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_matching

Independent of the module under test, nothing imported from slamlab: distances are Python
integers' bit counts, and matching is a brute force over sorted neighbour lists, with the
rotation histogram's maxima taken from a sort. Every value is an integer, so the policy is
``exact``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from tests.fixtures.foundations._layout import dumps

OUT = Path(__file__).with_name("matching.json")
EXACT = {"comparison": "exact", "rtol": 0.0, "atol": 0.0, "reason": "integer arithmetic"}
BYTES = 32
BINS = 30

Rows = list[list[int]]


def _distance(a: list[int], b: list[int]) -> int:
    """Count the differing bits of two descriptors."""
    return sum((x ^ y).bit_count() for x, y in zip(a, b, strict=True))


def _kept_bins(counts: list[int]) -> list[int]:
    """Return the histogram's highest three bins (ties: the lower), the lower two if large."""
    ranked = sorted(range(BINS), key=lambda k: (-counts[k], k))[:3]
    kept = [ranked[0]]
    for k in ranked[1:]:
        if counts[k] < 0.1 * counts[ranked[0]]:
            break
        kept.append(k)
    return kept


def _match(
    first: Rows,
    second: Rows,
    angles1: list[int],
    angles2: list[int],
    options: dict[str, Any],
) -> dict[str, Any]:
    """Match by brute force: sort each row's neighbours, filter, settle conflicts, check."""
    candidates = []
    for i, a in enumerate(first):
        ranked = sorted((_distance(a, b), j) for j, b in enumerate(second))
        if not ranked:
            continue
        best, j = ranked[0]
        ratio_ok = len(ranked) == 1 or best < options["ratio"] * ranked[1][0]
        if ratio_ok and best <= options["max_distance"]:
            candidates.append((j, best, i))
    winners: dict[int, tuple[int, int]] = {}
    for j, best, i in sorted(candidates):  # per second-set index, the nearest, then lowest i
        winners.setdefault(j, (i, best))
    kept = sorted((i, j, best) for j, (i, best) in winners.items())
    if options["check_orientation"] and kept:
        rotation = [(angles1[i] - angles2[j]) % BINS for i, j, _ in kept]
        counts = [rotation.count(k) for k in range(BINS)]
        bins = _kept_bins(counts)
        kept = [m for m, r in zip(kept, rotation, strict=True) if r in bins]
    return {"pairs": [[i, j] for i, j, _ in kept], "distances": [d for _, _, d in kept]}


def _planted(
    rng: np.random.Generator, n: int, extra: int, flips: int, turn: int
) -> tuple[Rows, Rows, list[int], list[int]]:
    """Two sets sharing `n` noisy copies, shuffled, plus `extra` unrelated descriptors each.

    True pairs differ by `flips` random bits and their orientations by `turn` bins; the
    unrelated ones have random orientations.
    """
    first = rng.integers(0, 256, (n + extra, BYTES))
    copies = first[:n].copy()
    for row in copies:
        bits = rng.choice(8 * BYTES, flips, replace=False)
        for bit in bits:
            row[bit // 8] ^= 1 << (bit % 8)
    second = np.vstack([copies, rng.integers(0, 256, (extra, BYTES))])
    angles1 = rng.integers(0, BINS, n + extra)
    angles2 = np.concatenate([(angles1[:n] + turn) % BINS, rng.integers(0, BINS, extra)])
    order = rng.permutation(len(second))
    return first.tolist(), second[order].tolist(), angles1.tolist(), angles2[order].tolist()


def main() -> None:
    """Write the fixture."""
    rng = np.random.default_rng(23)
    small = rng.integers(0, 256, (5, BYTES)).tolist()
    other = rng.integers(0, 256, (7, BYTES)).tolist()
    default = {"max_distance": 50, "ratio": 0.8, "check_orientation": True}
    match_cases = []
    for name, (first, second, a1, a2), options in [
        ("planted, rotated by 4 bins", _planted(rng, 40, 25, 12, 4), default),
        (
            "planted, no orientation check",
            _planted(rng, 30, 30, 20, 0),
            {**default, "check_orientation": False},
        ),
        ("copies exactly at the cap", _planted(rng, 20, 5, 30, 2), {**default, "max_distance": 30}),
        ("ratio 1: any strict nearest", _planted(rng, 10, 40, 30, 0), {**default, "ratio": 1.0}),
        ("one descriptor to match against", (small, small[2:3], [0] * 5, [0]), default),
        (
            "duplicates in the first set",
            (small + small, small, list(range(10)), list(range(5))),
            default,
        ),
        ("an empty set", (small, [], [1] * 5, []), default),
    ]:
        match_cases.append(
            {
                "name": name,
                "input": [first, second, a1, a2, options],
                "expected": _match(first, second, a1, a2, options),
            }
        )
    functions: dict[str, Any] = {
        "distances": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": name,
                    "input": [a, b],
                    "expected": [[_distance(x, y) for y in b] for x in a],
                }
                for name, a, b in [
                    ("random sets", small, other),
                    ("a set against itself", small, small),
                    ("all bits apart", [[0] * BYTES], [[255] * BYTES]),
                ]
            ],
        },
        "match": {"unpack": True, "tolerance": EXACT, "cases": match_cases},
    }
    fixture = {
        "description": "Descriptor matching: parity fixture (generate_matching.py).",
        "oracle": "bit counts and a brute force over sorted neighbour lists",
        "functions": functions,
    }
    OUT.write_text(dumps(fixture))


if __name__ == "__main__":
    main()
