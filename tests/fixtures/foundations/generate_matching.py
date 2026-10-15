"""Generate ``matching.json``, the descriptor-matching parity fixture.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_matching

Independent of the module under test, nothing imported from slam_it: distances are Python
integers' bit counts, and matching is a brute force over sorted neighbour lists, with the
rotation histogram's maxima taken from a sort; matching by word is the same brute force over
the second set's descriptors under the same node. A search by projection is a brute force too:
every keypoint tested against every window, its candidates sorted by (distance, index). Every
value is an integer, so the policy is ``exact``. A right column of ``null`` is none (NaN).
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
    *,
    nodes: tuple[list[int], list[int]] | None = None,
) -> dict[str, Any]:
    """Match by brute force: sort each row's neighbours, filter, settle conflicts, check.

    With `nodes`, a row's neighbours are only the second set's rows under its node.
    """
    candidates = []
    for i, a in enumerate(first):
        ranked = sorted(
            (_distance(a, b), j)
            for j, b in enumerate(second)
            if nodes is None or nodes[0][i] == nodes[1][j]
        )
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


def _in_area(
    xy: list[list[float]], level: list[int], window: tuple[float, float, float, int, int]
) -> list[int]:
    """Return the keypoints inside a window ``(u, v, r, min_level, max_level)``, ascending."""
    u, v, r, low, high = window
    return [
        k
        for k, ((x, y), lv) in enumerate(zip(xy, level, strict=True))
        if abs(x - u) < r and abs(y - v) < r and low <= lv <= high
    ]


def _search(
    projections: dict[str, Any], keypoints: dict[str, Any], options: dict[str, Any]
) -> dict[str, Any]:
    """Search by projection by brute force: each window's candidates sorted, then conflicts."""
    claim: dict[int, tuple[int, int]] = {}
    for i, (u, v) in enumerate(projections["xy"]):
        r = projections["radius"][i]
        window = (u, v, r, projections["min_level"][i], projections["max_level"][i])
        predicted = projections["right"][i]
        candidates = []
        for k in _in_area(keypoints["xy"], keypoints["level"], window):
            column = keypoints["right"][k]
            if keypoints["taken"][k]:
                continue
            if predicted is not None and column is not None and abs(column - predicted) > r:
                continue
            candidates.append(k)
        ranked = sorted(
            (_distance(projections["descriptors"][i], keypoints["descriptors"][k]), k)
            for k in candidates
        )
        if not ranked or ranked[0][0] > options["max_distance"]:
            continue
        best, k = ranked[0]
        if len(ranked) > 1:
            second, other = ranked[1]
            same_level = keypoints["level"][other] == keypoints["level"][k]
            if same_level and best > options["ratio"] * second:
                continue
        if k not in claim or best < claim[k][0]:
            claim[k] = (best, i)
    kept = sorted((i, k, d) for k, (d, i) in claim.items())
    if options["check_orientation"] and kept:
        rotation = [(projections["angle"][i] - keypoints["angle"][k]) % BINS for i, k, _ in kept]
        bins = _kept_bins([rotation.count(b) for b in range(BINS)])
        kept = [m for m, rot in zip(kept, rotation, strict=True) if rot in bins]
    return {"pairs": [[i, k] for i, k, _ in kept], "distances": [d for _, _, d in kept]}


def _flip(row: list[int], bits: list[int]) -> list[int]:
    """Return a descriptor with the given bits flipped."""
    out = list(row)
    for bit in bits:
        out[bit // 8] ^= 1 << (bit % 8)
    return out


def _frame(rng: np.random.Generator, n: int, levels: int = 8) -> dict[str, Any]:
    """Draw a frame's keypoints over a 640x480 image: no right columns, none taken."""
    level = rng.integers(0, levels, n).tolist()
    return {
        "xy": np.round(rng.uniform([0, 0], [640, 480], (n, 2)), 3).tolist(),
        "level": level,
        "angle": rng.integers(0, BINS, n).tolist(),
        "descriptors": rng.integers(0, 256, (n, BYTES)).tolist(),
        "right": [None] * n,
        "taken": [False] * n,
        "levels": levels,
    }


def _planted_projections(
    rng: np.random.Generator, frame: dict[str, Any], chosen: list[int], decoys: int, turn: int
) -> dict[str, Any]:
    """Project `chosen` keypoints back with noise and a few flipped bits, plus random decoys."""
    xy, radius, low, high, descriptors, angle = [], [], [], [], [], []
    for k in chosen:
        lv = frame["level"][k]
        xy.append([round(c + float(rng.uniform(-1.5, 1.5)), 3) for c in frame["xy"][k]])
        radius.append(round(4.0 * 1.2**lv, 6))
        low.append(lv - 1)
        high.append(lv + 1)
        flips = rng.choice(8 * BYTES, 6, replace=False).tolist()
        descriptors.append(_flip(frame["descriptors"][k], flips))
        angle.append((frame["angle"][k] + turn) % BINS)
    for _ in range(decoys):
        xy.append(np.round(rng.uniform([0, 0], [640, 480]), 3).tolist())
        radius.append(10.0)
        low.append(0)
        high.append(7)
        descriptors.append(rng.integers(0, 256, BYTES).tolist())
        angle.append(int(rng.integers(0, BINS)))
    m = len(xy)
    return {
        "xy": xy,
        "radius": radius,
        "min_level": low,
        "max_level": high,
        "descriptors": descriptors,
        "angle": angle,
        "right": [None] * m,
    }


def _one(
    xy: list[float], radius: float, levels: tuple[int, int], descriptor: list[int], angle: int = 0
) -> dict[str, Any]:
    """One projection, as a one-row projections dict (no right column)."""
    return {
        "xy": [xy],
        "radius": [radius],
        "min_level": [levels[0]],
        "max_level": [levels[1]],
        "descriptors": [descriptor],
        "angle": [angle],
        "right": [None],
    }


def _stack(*projections: dict[str, Any]) -> dict[str, Any]:
    """Concatenate projections dicts, row for row."""
    return {key: [row for p in projections for row in p[key]] for key in projections[0]}


def _keypoints(xy: list[list[float]], level: list[int], descriptors: Rows) -> dict[str, Any]:
    """Return a small hand-made frame: angles 0, no right columns, none taken, 8 levels."""
    n = len(xy)
    return {
        "xy": xy,
        "level": level,
        "angle": [0] * n,
        "descriptors": descriptors,
        "right": [None] * n,
        "taken": [False] * n,
        "levels": 8,
    }


def _search_cases(rng: np.random.Generator) -> list[dict[str, Any]]:
    """Return the search-by-projection cases: planted frames, then each filter by hand."""
    last_frame = {"max_distance": 100, "ratio": 1.0, "check_orientation": True}
    local_map = {"max_distance": 100, "ratio": 0.8, "check_orientation": False}
    frame = _frame(rng, 300)
    picked = rng.choice(300, 86, replace=False).tolist()
    chosen, others = sorted(picked[:80]), sorted(picked[80:])
    planted = _planted_projections(rng, frame, chosen, 15, 3)
    stray = _planted_projections(rng, frame, others, 0, 17)  # 6, under a tenth of the bulk
    base = rng.integers(0, 256, BYTES).tolist()
    near, nearer = _flip(base, [0, 1, 2]), _flip(base, [0])
    far = _flip(base, list(range(0, 200, 8)))  # 25 bits off
    # Two candidates on one level at 20 and 22 bits: 20 > 0.8 * 22, so the ratio test rejects;
    # on two levels it does not apply.
    twenty, twenty_two = _flip(base, list(range(20))), _flip(base, list(range(40, 62)))
    ambiguous = _keypoints([[100.0, 100.0], [102.0, 100.0]], [2, 2], [twenty, twenty_two])
    levelled = _keypoints([[100.0, 100.0], [102.0, 100.0]], [2, 3], [twenty, twenty_two])
    stereo = _keypoints(
        [[200.0, 50.0], [201.0, 50.0], [202.0, 50.0]], [0, 0, 0], [nearer, near, far]
    )
    stereo["right"] = [150.0, 180.0, None]
    stereo_projection = _one([201.0, 50.0], 5.0, (0, 1), base)
    stereo_projection["right"] = [175.0]  # 25 from the nearest, 5 from the next: kept at r
    taken = _keypoints([[10.0, 10.0], [11.0, 10.0]], [0, 0], [nearer, near])
    taken["taken"] = [True, False]
    edges = _keypoints(
        [[50.0, 50.0], [54.0, 50.0], [50.0, 53.999], [46.0, 50.0]], [1, 1, 1, 1],
        [nearer, nearer, near, nearer],
    )  # fmt: skip
    conflict = _keypoints([[300.0, 300.0]], [4], [base])
    between = _one([101.0, 100.0], 5.0, (0, 7), base)
    beside = _one([10.5, 10.0], 3.0, (0, 0), base)
    at_edges = _one([50.0, 50.0], 4.0, (1, 1), base)
    missing = _one([50.0, 50.0], 4.0, (2, 7), base)
    first, second = _one([300.0, 300.0], 5.0, (3, 5), near), _one([301.0, 300.0], 5.0, (3, 5), near)
    nearer_second = _one([301.0, 300.0], 5.0, (3, 5), nearer)
    none: dict[str, list[Any]] = {key: [] for key in planted}
    capped, below_cap = {**local_map, "max_distance": 6}, {**local_map, "max_distance": 5}
    cases: list[tuple[str, dict[str, Any], dict[str, Any], dict[str, Any]]] = [
        ("planted, from the last frame", planted, frame, last_frame),
        (
            "planted, with strays the rotation check drops",
            _stack(planted, stray),
            frame,
            last_frame,
        ),
        ("planted, the local map's filters", planted, frame, local_map),
        ("a distance cap at the copies' distance", planted, frame, capped),
        ("a distance cap just below it", planted, frame, below_cap),
        ("the ratio test on one level", between, ambiguous, local_map),
        ("no ratio test across levels", between, levelled, local_map),
        ("ratio 1 never rejects", between, ambiguous, last_frame),
        ("predicted right columns", stereo_projection, stereo, last_frame),
        ("taken keypoints are never candidates", beside, taken, last_frame),
        ("window edges are strict", at_edges, edges, last_frame),
        ("a level range that misses", missing, edges, last_frame),
        (
            "two projections, one keypoint: the nearer keeps it",
            _stack(first, nearer_second),
            conflict,
            last_frame,
        ),
        ("two projections tied: the earlier keeps it", _stack(first, second), conflict, last_frame),
        ("no projections", none, frame, last_frame),
        ("no keypoints", planted, _keypoints([], [], []), last_frame),
    ]
    return [
        {
            "name": name,
            "input": [projections, keypoints, options],
            "expected": _search(projections, keypoints, options),
        }
        for name, projections, keypoints, options in cases
    ]


def _area_cases(rng: np.random.Generator) -> list[dict[str, Any]]:
    """Return the window cases: a random frame's keypoints under several windows."""
    frame = _frame(rng, 400)
    same_x = [[123.5, float(y)] for y in range(0, 480, 40)]
    column = {"xy": same_x, "level": [0] * len(same_x)}
    windows = [
        ("a small window on every level", frame, (320.0, 240.0, 40.0, 0, 7)),
        ("a window on two levels", frame, (100.0, 380.0, 60.0, 2, 3)),
        ("a window over the whole image", frame, (320.0, 240.0, 1e6, 0, 7)),
        ("a window past the image's corner", frame, (-5.0, 490.0, 60.0, 0, 7)),
        ("an empty level range", frame, (320.0, 240.0, 1e6, 5, 4)),
        ("keypoints in one column", column, (123.5, 200.0, 50.0, 0, 0)),
        ("no keypoints", {"xy": [], "level": []}, (10.0, 10.0, 5.0, 0, 7)),
    ]  # fmt: skip
    return [
        {
            "name": name,
            "input": [keys["xy"], keys["level"], [u, v], r, low, high],
            "expected": _in_area(keys["xy"], keys["level"], (u, v, r, low, high)),
        }
        for name, keys, (u, v, r, low, high) in windows
    ]


def _bow_cases() -> list[dict[str, Any]]:
    """Return matching by word's cases: pairs under shared nodes, some split, lone, none.

    Drawn from their own generator, so that the other functions' cases stay as they were.
    """
    rng = np.random.default_rng(29)
    default = {"max_distance": 50, "ratio": 0.75, "check_orientation": True}
    first, second, a1, a2 = _planted(rng, 40, 25, 12, 3)
    # The copies of the first set's first 40 share their node; the rest are drawn at random.
    nodes1 = rng.integers(0, 6, len(first)).tolist()
    nodes2 = rng.integers(0, 6, len(second)).tolist()
    planted = rng.integers(0, 6, 40).tolist()
    for i in range(40):
        nodes1[i] = planted[i]
    for j, row in enumerate(second):
        twin = next((i for i in range(40) if _distance(first[i], row) <= 12), None)
        if twin is not None:
            nodes2[j] = planted[twin]
    split = list(nodes2)
    for j in range(0, len(second), 3):
        split[j] = 7  # a third of the second set's descriptors under a node the first lacks
    small = rng.integers(0, 256, (6, BYTES)).tolist()
    cases = [
        ("planted pairs under shared nodes", (first, nodes1, second, nodes2, a1, a2), default),
        ("a third of them under another node", (first, nodes1, second, split, a1, a2), default),
        (
            "every descriptor under one node: match's result",
            (first, [0] * len(first), second, [0] * len(second), a1, a2),
            default,
        ),
        (
            "a node with one descriptor to match against",
            (small, [1, 1, 2, 2, 3, 3], small[:1] + small[3:4], [1, 2], [0] * 6, [0, 0]),
            default,
        ),
        ("no node shared", (small, [0] * 6, small, [1] * 6, [0] * 6, [0] * 6), default),
        ("an empty set", (small, [0] * 6, [], [], [0] * 6, []), default),
    ]
    return [
        {
            "name": name,
            "input": [d1, n1, d2, n2, b1, b2, options],
            "expected": _match(d1, d2, b1, b2, options, nodes=(n1, n2)),
        }
        for name, (d1, n1, d2, n2, b1, b2), options in cases
    ]


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
        "features_in_area": {"unpack": True, "tolerance": EXACT, "cases": _area_cases(rng)},
        "search_by_projection": {"unpack": True, "tolerance": EXACT, "cases": _search_cases(rng)},
        "search_by_bow": {"unpack": True, "tolerance": EXACT, "cases": _bow_cases()},
    }
    fixture = {
        "description": "Descriptor matching: parity fixture (generate_matching.py).",
        "oracle": "bit counts and a brute force over sorted neighbour lists",
        "functions": functions,
    }
    OUT.write_text(dumps(fixture))


if __name__ == "__main__":
    main()
