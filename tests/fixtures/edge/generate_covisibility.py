"""Generate ``covisibility.json``: the covisibility graph's parity fixture, by brute force.

Run from the repository root::

    python -m tests.fixtures.edge.generate_covisibility

Each case is a map given as its keyframes, each with its id, the point each keypoint observes
(-1 for none) and its parent, with the covisibility settings it is read with; and what
:mod:`slam_it.edge.covisibility` must give on it, computed here from sets, by the rules alone
and none of the module's code:

- each keyframe's links and their weights;
- tracking's local map for some frames' matched points;
- each keyframe's parent from :func:`~slam_it.edge.covisibility.adopt`, with its own unset;
- each keyframe's children given new parents by :func:`~slam_it.edge.covisibility.reparent`;
- the strong links.

Two cases are built by hand: ties between equal weights and the link kept when none reaches
``min_points``; and a culled keyframe whose children are joined in rounds, one becoming the
parent of the next, ids not their rows. The rest are maps of points seen along a path, from a
fixed seed, read with the local map's caps both far and near. The Python, C++ and Rust tests
hold their backends to the file (``tests/edge/test_covisibility.py``,
``cpp/tests/edge/test_covisibility.cpp``, ``rust/slam-it-edge/tests/covisibility.rs``).
"""

from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Any

HERE = Path(__file__).parent

Keyframes = list[dict[str, Any]]


def _points(keyframe: dict[str, Any]) -> set[int]:
    return {p for p in keyframe["points"] if p >= 0}


def _weight(keyframes: Keyframes, a: int, b: int) -> int:
    return 0 if a == b else len(_points(keyframes[a]) & _points(keyframes[b]))


def _links(keyframes: Keyframes, a: int, min_points: int, without: int = -1) -> list[list[int]]:
    """Return the rows linked to `a` and their weights, `without`'s links dropped (a row)."""
    shared = [
        (_weight(keyframes, a, b), keyframes[b]["id"], b)
        for b in range(len(keyframes))
        if b not in (a, without) and _weight(keyframes, a, b) > 0
    ]
    shared.sort(key=lambda s: (-s[0], s[1]))
    kept = [s for s in shared if s[0] >= min_points] or shared[:1]
    return [[s[2] for s in kept], [s[0] for s in kept]]


def _row(keyframes: Keyframes, keyframe_id: int) -> int:
    return next((r for r, k in enumerate(keyframes) if k["id"] == keyframe_id), -1)


def _children(keyframes: Keyframes, a: int) -> list[int]:
    return [r for r, k in enumerate(keyframes) if k["parent"] == keyframes[a]["id"]]


ADDED: dict[str, int] = {"neighbour": 0, "child": 0, "parent": 0}
"""How many keyframes K2 took each way, over the fixture: each must be tested."""


def _local_map(
    keyframes: Keyframes, matched: list[int], settings: dict[str, int]
) -> dict[str, Any]:
    seen = {p for p in matched if p >= 0}
    counts = [(len(seen & _points(k)), k["id"], r) for r, k in enumerate(keyframes)]
    k1 = [r for _, _, r in sorted((c for c in counts if c[0] > 0), key=lambda c: (-c[0], c[1]))]
    if not k1:
        return {"keyframes": [], "reference": -1}
    local = list(k1)
    for k in k1:
        if len(local) > settings["local_keyframes"]:
            break
        best = _links(keyframes, k, settings["min_points"])[0][: settings["local_neighbours"]]
        for r in best:
            if r not in local:
                local.append(r)
                ADDED["neighbour"] += 1
                break
        for r in _children(keyframes, k):
            if r not in local:
                local.append(r)
                ADDED["child"] += 1
                break
        parent = _row(keyframes, keyframes[k]["parent"])
        if parent >= 0 and parent not in local:
            local.append(parent)
            ADDED["parent"] += 1
    return {"keyframes": local, "reference": k1[0]}


def _adopted(keyframes: Keyframes, a: int) -> int:
    """Return the parent `a` takes, its own unset: the one sharing most, ties by the lower id."""
    if keyframes[a]["id"] == 0:
        return -1
    shared = [(_weight(keyframes, a, b), keyframes[b]["id"]) for b in range(len(keyframes))]
    best = sorted((s for s in shared if s[0] > 0), key=lambda s: (-s[0], s[1]))
    return best[0][1] if best else -1


def _reparented(keyframes: Keyframes, culled: int, min_points: int) -> list[int]:
    """Return every keyframe's parent after `culled`'s children get new ones (SetBadFlag)."""
    parents = [k["parent"] for k in keyframes]
    children = _children(keyframes, culled)
    own = keyframes[culled]["parent"]
    candidates = {_row(keyframes, own)} - {-1}
    linked = {c: _links(keyframes, c, min_points, without=culled) for c in children}
    while children:
        best = None
        for c in children:
            for r, w in zip(*linked[c], strict=True):
                if r in candidates and (best is None or w > best[0]):
                    best = (w, c, r)
        if best is None:
            break
        _, c, r = best
        parents[c] = keyframes[r]["id"]
        candidates.add(c)
        children.remove(c)
    for c in children:
        parents[c] = own
    return parents


def _strong(keyframes: Keyframes, strong_points: int) -> list[list[int]]:
    return [
        [keyframes[a]["id"], keyframes[b]["id"], _weight(keyframes, a, b)]
        for a in range(len(keyframes))
        for b in range(a + 1, len(keyframes))
        if _weight(keyframes, a, b) >= strong_points
    ]


def _case(
    name: str, keyframes: Keyframes, settings: dict[str, int], frames: list[list[int]]
) -> dict[str, Any]:
    rows = range(len(keyframes))
    return {
        "name": name,
        "keyframes": keyframes,
        "settings": settings,
        "links": [_links(keyframes, a, settings["min_points"]) for a in rows],
        "local_maps": [
            {"points": points, **_local_map(keyframes, points, settings)} for points in frames
        ],
        "adopted": [_adopted(keyframes, a) for a in rows],
        "reparented": [_reparented(keyframes, a, settings["min_points"]) for a in rows],
        "strong_links": _strong(keyframes, settings["strong_points"]),
    }


def _kf(keyframe_id: int, points: list[int], parent: int) -> dict[str, Any]:
    return {"id": keyframe_id, "points": points, "parent": parent}


def ties() -> dict[str, Any]:
    """Return a case of equal weights ranked by id, and a keyframe sharing too few."""
    keyframes = [
        _kf(0, list(range(10)), -1),
        _kf(1, [0, 1, 2, 3, 4, 10, 11, -1], 0),
        _kf(2, [5, 6, 7, 8, 9, 10, 12], 0),
        _kf(3, [0, 12, 13, -1, -1], 0),
        _kf(5, [13], 3),
    ]
    settings = {"min_points": 5, "local_keyframes": 80, "local_neighbours": 10, "strong_points": 5}
    return _case("ties", keyframes, settings, [[0, 1, 13, -1], [12], [-1, -1], [99]])


def repair() -> dict[str, Any]:
    """Return a case whose culled keyframe's children join in rounds, ids not their rows."""
    keyframes = [
        _kf(0, list(range(20)), -1),
        _kf(1, [*range(10), *range(20, 30)], 0),
        _kf(2, [*range(20, 25), *range(30, 40)], 1),
        _kf(4, [*range(30, 45)], 1),
        _kf(6, [5, 6, 7, 30, 31, 32, *range(45, 50)], 1),
    ]
    settings = {"min_points": 3, "local_keyframes": 80, "local_neighbours": 10, "strong_points": 10}
    case = _case("repair", keyframes, settings, [[0, 20, 30, 45]])
    # Culling id 1: id 6 joins its parent 0, then id 2 joins id 6, then id 4 joins id 2.
    assert case["reparented"][1] == [-1, 0, 6, 2, 0]
    return case


def path(seed: int, settings: dict[str, int]) -> dict[str, Any]:
    """Return keyframes along a path, each seeing a stretch of the points, parents as inserted."""
    rng = random.Random(seed)
    keyframes: Keyframes = []
    next_id = 0
    for i in range(14):
        start = 4 * i + rng.randrange(3)
        seen = [p for p in range(start, start + rng.randrange(8, 16)) if rng.random() < 0.8]
        seen += [-1] * rng.randrange(3)
        rng.shuffle(seen)
        keyframes.append(_kf(next_id, seen, -1))
        keyframes[-1]["parent"] = _adopted(keyframes, len(keyframes) - 1)  # as inserted
        next_id += 1 + (rng.random() < 0.25)  # some ids skipped, as culling leaves them
    frames = [[p if rng.random() < 0.6 else -1 for p in range(s, s + 12)] for s in (0, 17, 30, 49)]
    return _case(f"path-{seed}", keyframes, settings, frames)


def main() -> None:
    """Write the fixture."""
    near = {"min_points": 3, "local_keyframes": 4, "local_neighbours": 2, "strong_points": 6}
    far = {"min_points": 2, "local_keyframes": 80, "local_neighbours": 10, "strong_points": 8}
    cases = [ties(), repair(), path(1, far), path(2, near), path(3, near)]
    capped = [m for c in cases[2:] for m in c["local_maps"] if len(m["keyframes"]) > 4]
    assert capped, "no local map reached its cap"
    assert all(ADDED.values()), f"K2 never took a keyframe some way: {ADDED}"
    doc = {
        "description": "The covisibility graph, by brute force (generate_covisibility.py).",
        "cases": cases,
    }
    (HERE / "covisibility.json").write_text(json.dumps(doc, indent=1) + "\n")


if __name__ == "__main__":
    main()
