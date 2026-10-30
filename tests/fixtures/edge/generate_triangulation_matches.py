"""Generate ``triangulation_matches.json``: matching for triangulation's parity fixture.

Run from the repository root::

    python -m tests.fixtures.edge.generate_triangulation_matches

Each case is a map of two keyframes, the newest (``k1``) matched against the other (``k2``) as
:func:`slam_it.edge.mapping.match_for_triangulation` matches them: each keyframe's pose
``T_cam_world`` and, for each keypoint, its bearing, pyramid level, depth (null for none),
descriptor (its 32 bytes), vocabulary node and the point it observes (-1 for none); the map's
points, the camera's focal length, the pyramid and the settings; and the matches the rules give,
as keypoint rows ``(in k1, in k2)`` by ``k1`` keypoint, computed here pair by pair from the rules
alone and none of the module's code:

- a pair is a candidate if both keypoints observe no point and have a bearing in front, they are
  under the same vocabulary node (ORB-SLAM3's ``SearchForTriangulation`` walks the two keyframes'
  ``FeatureVector`` node by node), their descriptors are within ``matching.th_low``, the second
  lies within ``mapping.epipolar_chi2`` of its level's variance of the first's epipolar line (in
  pixels at the focal length) and, when neither has depth, it lies no nearer the epipole than
  ``mapping.min_epipole_distance2`` scaled by its level;
- each ``k1`` keypoint takes its nearest candidate, the last (the highest row) among equals, as
  ORB-SLAM3's ``dist > bestDist`` skips only a farther one;
- each ``k2`` keypoint taken goes to the nearest ``k1`` keypoint that took it, the lower among
  equals.

Hand-built cases make each rule decide somewhere (asserted): a near descriptor under another
node, one off the epipolar line, one near the epipole without depth and with it, a keypoint
observing a point, one behind its camera, a descriptor past ``matching.th_low``, two candidates
at the same distance, and two ``k1`` keypoints taking the same ``k2`` keypoint, nearer and at the
same distance. Random scenes from fixed seeds follow, their keypoints shuffled. No pair lies
within a factor of 2 of the epipolar band or the epipole's bound, so the backends' rounding
cannot move a decision. The Python, C++ and Rust tests hold their backends to the file
(``tests/edge/test_mapping.py``, ``cpp/tests/edge/test_mapping.cpp``,
``rust/slam-it-edge/tests/mapping.rs``).
"""

from __future__ import annotations

import json
import math
import random
from pathlib import Path
from typing import Any

HERE = Path(__file__).parent

FOCAL = 500.0
"""The camera's focal length, in pixels."""

LEVELS, SCALE_FACTOR = 8, 1.2
"""The pyramid: ORB-SLAM's."""

SETTINGS = {
    "matching.th_low": 50,
    "mapping.epipolar_chi2": 3.84,
    "mapping.min_epipole_distance2": 100.0,
}
"""The settings the cases are read with: the defaults."""

BYTES = 32
"""A descriptor's bytes."""

MARGIN = 2.0
"""How far every pair stays from a bound: outside [bound / MARGIN, bound * MARGIN]."""

Vector = list[float]
Matrix = list[list[float]]


def _rotation_y(angle: float) -> Matrix:
    c, s = math.cos(angle), math.sin(angle)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _pose(rotation: Matrix, centre: Vector) -> Matrix:
    """Return ``T_cam_world`` of a camera with this rotation, at this centre."""
    t = [-sum(rotation[r][c] * centre[c] for c in range(3)) for r in range(3)]
    return [[*rotation[r], t[r]] for r in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _to_camera(pose: Matrix, x: Vector) -> Vector:
    return [sum(pose[r][c] * x[c] for c in range(3)) + pose[r][3] for r in range(3)]


def _unit(v: Vector) -> Vector:
    n = math.sqrt(sum(a * a for a in v))
    return [a / n for a in v]


def _cross(a: Vector, b: Vector) -> Vector:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _distance(a: str, b: str) -> int:
    return bin(int(a, 16) ^ int(b, 16)).count("1")


def _flipped(descriptor: str, bits: list[int]) -> str:
    value = int(descriptor, 16)
    for b in bits:
        value ^= 1 << b
    return f"{value:0{2 * BYTES}x}"


def _random_descriptor(rng: random.Random) -> str:
    return f"{rng.getrandbits(8 * BYTES):0{2 * BYTES}x}"


class Keyframe:
    """A keyframe being built: its pose and keypoints, column by column."""

    def __init__(self, pose: Matrix) -> None:
        self.pose = pose
        self.bearing: list[Vector] = []
        self.level: list[int] = []
        self.depth: list[float | None] = []
        self.descriptor: list[str] = []
        self.node: list[int] = []
        self.point: list[int] = []

    def add(
        self,
        bearing: Vector,
        level: int,
        depth: float | None,
        descriptor: str,
        node: int,
        *,
        point: int = -1,
    ) -> int:
        """Add a keypoint; return its index in the keyframe."""
        self.bearing.append(bearing)
        self.level.append(level)
        self.depth.append(depth)
        self.descriptor.append(descriptor)
        self.node.append(node)
        self.point.append(point)
        return len(self.bearing) - 1

    def see(self, x: Vector, level: int, *, with_depth: bool, descriptor: str, node: int) -> int:
        """Add the keypoint of a world point, its depth the point's if asked."""
        local = _to_camera(self.pose, x)
        return self.add(_unit(local), level, local[2] if with_depth else None, descriptor, node)

    def as_dict(self) -> dict[str, Any]:
        """Return the keyframe as the fixture holds it, each descriptor as its bytes."""
        return {
            "pose": self.pose,
            "bearing": self.bearing,
            "level": self.level,
            "depth": self.depth,
            "descriptor": [list(bytes.fromhex(d)) for d in self.descriptor],
            "node": self.node,
            "point": self.point,
        }


def _relative(first: Keyframe, second: Keyframe) -> tuple[Matrix, Vector]:
    """Return ``T_2_1``'s rotation and translation: ``R2 R1^T`` and ``t2 - R t1``."""
    r1 = [row[:3] for row in first.pose[:3]]
    r2 = [row[:3] for row in second.pose[:3]]
    t1 = [first.pose[r][3] for r in range(3)]
    t2 = [second.pose[r][3] for r in range(3)]
    rotation = [[sum(r2[i][k] * r1[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    moved = [sum(rotation[i][k] * t1[k] for k in range(3)) for i in range(3)]
    return rotation, [t2[i] - moved[i] for i in range(3)]


def _geometry(k1: Keyframe, k2: Keyframe, i: int, j: int) -> tuple[float, float | None]:
    """Return a pair's epipolar distance squared (pixels) and, with an epipole, its distance."""
    rotation, t = _relative(k1, k2)
    b1, b2 = k1.bearing[i], k2.bearing[j]
    x1 = [b1[0] / b1[2], b1[1] / b1[2], 1.0]
    x2 = [b2[0] / b2[2], b2[1] / b2[2], 1.0]
    rx1 = [sum(rotation[r][c] * x1[c] for c in range(3)) for r in range(3)]
    line = _cross(t, rx1)  # E x1, E = [t]x R
    numerator = sum(line[c] * x2[c] for c in range(3))
    band = numerator * numerator / (line[0] * line[0] + line[1] * line[1]) * FOCAL * FOCAL
    if t[2] == 0.0:
        return band, None
    epipole = [t[0] / t[2], t[1] / t[2]]
    near = ((x2[0] - epipole[0]) ** 2 + (x2[1] - epipole[1]) ** 2) * FOCAL * FOCAL
    return band, near


RULES = ("node", "band", "epipole", "observed", "behind", "distance")
"""The rules a pair may fail, each asserted to decide some pair."""

DECIDED: dict[str, int] = dict.fromkeys(RULES, 0)
"""How many pairs each rule alone refused, over the fixture."""

EVENTS: dict[str, int] = {"tie": 0, "claimed": 0, "claimed_tie": 0, "depth_spares": 0}
"""Ties between candidates, and k2 keypoints two k1 keypoints took (nearer, and equal)."""


def _candidate(case: dict[str, Any], k1: Keyframe, k2: Keyframe, i: int, j: int) -> int | None:
    """Return a pair's distance if it is a candidate, None otherwise (the module docstring)."""
    failed = []
    if k1.point[i] >= 0 or k2.point[j] >= 0:
        failed.append("observed")
    if not (k1.bearing[i][2] > 0.0 and k2.bearing[j][2] > 0.0):
        failed.append("behind")
        if len(failed) == 1:
            DECIDED["behind"] += 1
        return None  # no epipolar line to test
    if k1.node[i] != k2.node[j]:
        failed.append("node")
    d = _distance(k1.descriptor[i], k2.descriptor[j])
    if d > case["settings"]["matching.th_low"]:
        failed.append("distance")
    band, near = _geometry(k1, k2, i, j)
    bound = case["settings"]["mapping.epipolar_chi2"] * SCALE_FACTOR ** (2 * k2.level[j])
    no_depth = k1.depth[i] is None and k2.depth[j] is None
    if not ("observed" in failed or "distance" in failed):
        assert not bound / MARGIN < band < bound * MARGIN, (case["name"], i, j, band, bound)
    if band >= bound:
        failed.append("band")
    if near is not None:
        _epipole(case, failed, near, k2.level[j], no_depth=no_depth)
    if len(failed) == 1:
        DECIDED[failed[0]] += 1
    return None if failed else d


def _epipole(
    case: dict[str, Any], failed: list[str], near: float, level: int, *, no_depth: bool
) -> None:
    """Add the epipole's rule to a pair's failures: near it without depth (the docstring)."""
    limit = case["settings"]["mapping.min_epipole_distance2"] * SCALE_FACTOR**level
    if not ("observed" in failed or "distance" in failed or "band" in failed):
        assert not limit / MARGIN < near < limit * MARGIN, (case["name"], near, limit)
    if near < limit:
        if no_depth:
            failed.append("epipole")
        elif not failed:
            EVENTS["depth_spares"] += 1


def _matches(case: dict[str, Any], k1: Keyframe, k2: Keyframe) -> list[list[int]]:
    """Return the matches by the rules, as map keypoint rows ``(in k1, in k2)`` by k1 keypoint."""
    claims: dict[int, tuple[int, int]] = {}
    for i in range(len(k1.bearing)):
        best: tuple[int, int] | None = None
        for j in range(len(k2.bearing)):
            d = _candidate(case, k1, k2, i, j)
            if d is None:
                continue
            if best is not None and d == best[0]:
                EVENTS["tie"] += 1
            if best is None or d <= best[0]:
                best = (d, j)
        if best is None:
            continue
        d, j = best
        if j in claims:
            EVENTS["claimed_tie" if d == claims[j][0] else "claimed"] += 1
        if j not in claims or d < claims[j][0]:
            claims[j] = (d, i)
    start = case["starts"]
    return sorted([start[1] + i, start[0] + j] for j, (_, i) in claims.items())


def _case(name: str, k2: Keyframe, k1: Keyframe, points: list[Vector]) -> dict[str, Any]:
    """Return a case: keyframe 0 is ``k2``, keyframe 1 (the newest) ``k1``."""
    case: dict[str, Any] = {
        "name": name,
        "focal": FOCAL,
        "levels": LEVELS,
        "scale_factor": SCALE_FACTOR,
        "settings": SETTINGS,
        "keyframes": [k2.as_dict(), k1.as_dict()],
        "points": points,
        "k1": 1,
        "k2": 0,
        "starts": [0, len(k2.bearing)],
    }
    case["pairs"] = _matches(case, k1, k2)
    del case["starts"]
    return case


def _forward() -> tuple[Keyframe, Keyframe]:
    """Return two empty keyframes: the first at the origin, the second moved forward and aside."""
    first = Keyframe(_pose(_rotation_y(0.0), [0.0, 0.0, 0.0]))
    second = Keyframe(_pose(_rotation_y(0.05), [0.3, 0.05, 0.4]))
    return first, second


def hand_built() -> dict[str, Any]:
    """Return the case where each rule decides: the module docstring's list, in order."""
    rng = random.Random(7)
    k2, k1 = _forward()
    base = [_random_descriptor(rng) for _ in range(12)]

    def pair(x: Vector, d: int, flips: int, **kw: Any) -> None:  # noqa: ANN401 -- keypoint options
        """Add a world point seen by both, its k1 descriptor `flips` bits from its k2 one."""
        k2.see(
            x, kw.get("level2", 0), with_depth=kw.get("depth2", False), descriptor=base[d],
            node=kw.get("node2", 1),
        )  # fmt: skip
        k1.see(
            x, kw.get("level1", 0), with_depth=kw.get("depth1", False),
            descriptor=_flipped(base[d], list(range(flips))), node=kw.get("node1", 1),
        )  # fmt: skip

    pair([0.5, 0.2, 4.0], 0, 10)  # a match
    pair([-0.6, 0.1, 5.0], 1, 10, node1=2)  # another node: none
    pair([0.2, -0.4, 3.0], 2, 51)  # past matching.th_low: none
    pair([-0.3, -0.3, 6.0], 3, 20, level1=2, level2=3)  # a match at coarser levels
    # Off the epipolar line: k2 sees the point, k1 a keypoint well off its projection.
    x = [0.8, -0.2, 4.5]
    k2.see(x, 0, with_depth=False, descriptor=base[4], node=1)
    off = _to_camera(k1.pose, x)
    k1.add(_unit([off[0] + 0.2, off[1], off[2]]), 0, None, _flipped(base[4], [0, 1]), 1)
    # Near the epipole, without depth and with it: points just off the baseline's line, far
    # ahead, a few pixels from where k2 sees k1's centre (on the line, a keypoint would have no
    # epipolar line).
    centre = [0.3, 0.05, 0.4]  # k1's centre: k2 sees it at the epipole
    pair([centre[0] * 12.0 + 0.05, centre[1] * 12.0, centre[2] * 12.0], 5, 5)  # refused
    pair([centre[0] * 14.0 + 0.03, centre[1] * 14.0, centre[2] * 14.0], 6, 5, depth1=True)
    # A keypoint observing a point, and one behind its camera.
    x = [-0.1, 0.5, 3.5]
    k2.see(x, 0, with_depth=False, descriptor=base[7], node=1)
    local = _to_camera(k1.pose, x)
    k1.add(_unit(local), 0, None, _flipped(base[7], [3]), 1, point=0)
    k2.add([0.0, 0.0, -1.0], 0, None, base[8], 1)
    k1.add(_unit([0.01, 0.0, 1.0]), 0, None, _flipped(base[8], [5]), 1)
    # Two k2 candidates at the same distance on one k1 keypoint's epipolar line: the last wins.
    x1 = [0.4, 0.4, 3.0]
    b1 = _to_camera(k1.pose, x1)
    k1.add(_unit(b1), 0, None, base[9], 1)
    inverse = _inverse(k1.pose)
    for depth, bits in ((2.0, [0, 1, 2]), (5.0, [3, 4, 5])):  # two points on its ray
        on_ray = _to_camera(inverse, [b1[0] / b1[2] * depth, b1[1] / b1[2] * depth, depth])
        k2.see(on_ray, 0, with_depth=False, descriptor=_flipped(base[9], bits), node=1)
    # Two k1 keypoints taking the same k2 keypoint: the nearer keeps it; then at equal distance,
    # the lower.
    for d, (near_bits, far_bits) in ((10, ([0, 1], [0, 1, 2, 3])), (11, ([0, 1], [2, 3]))):
        x2 = [-0.5 - 0.1 * (d - 10), 0.3, 4.0]
        b2 = _to_camera(k2.pose, x2)
        k2.add(_unit(b2), 0, None, base[d], 1)
        for depth, bits in ((3.0, near_bits), (6.0, far_bits)):
            on_ray = [
                b2[0] / b2[2] * depth,
                b2[1] / b2[2] * depth,
                depth,
            ]  # k2's frame is the world
            k1.see(on_ray, 0, with_depth=False, descriptor=_flipped(base[d], bits), node=1)
    return _case("rules", k2, k1, [[-0.1, 0.5, 3.5]])


def _inverse(pose: Matrix) -> Matrix:
    """Return ``T_world_cam`` from ``T_cam_world``."""
    r = [row[:3] for row in pose[:3]]
    t = [pose[i][3] for i in range(3)]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    shift = [-sum(rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [[*rt[i], shift[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def scene(seed: int, *, sideways: bool) -> dict[str, Any]:
    """Return a random scene: shared points, each under a node, some with depth, and clutter.

    `sideways` moves the second keyframe across, its centre on the first's focal plane, so the
    epipole is at infinity and only the band decides.
    """
    rng = random.Random(seed)
    k2 = Keyframe(_pose(_rotation_y(0.0), [0.0, 0.0, 0.0]))
    centre = [0.4, -0.1, 0.0] if sideways else [0.2, 0.1, 0.3]
    turn = 0.0 if sideways else rng.uniform(-0.1, 0.1)  # sideways: no epipole
    k1 = Keyframe(_pose(_rotation_y(turn), centre))
    epipole = None if sideways else [centre[0] / centre[2], centre[1] / centre[2]]
    shared = []
    for _ in range(40):
        x = [rng.uniform(-2.0, 2.0), rng.uniform(-1.5, 1.5), rng.uniform(2.0, 9.0)]
        if _to_camera(k1.pose, x)[2] < 1.0:
            continue
        if epipole and math.dist([x[0] / x[2], x[1] / x[2]], epipole) * FOCAL < 30.0:
            continue  # the hand-built case tests the epipole; none here near its bound
        node = rng.randrange(6)
        shared.append((x, _random_descriptor(rng), node, node if rng.random() < 0.8 else 6))
    order1, order2 = list(range(len(shared))), list(range(len(shared)))
    rng.shuffle(order1)
    rng.shuffle(order2)
    for k in order2:
        x, descriptor, node, _ = shared[k]
        level, with_depth = rng.randrange(LEVELS), rng.random() < 0.5
        k2.see(x, level, with_depth=with_depth, descriptor=descriptor, node=node)
    for k in order1:
        x, descriptor, _, node1 = shared[k]
        flips = rng.sample(range(8 * BYTES), rng.randrange(0, 61))
        level, with_depth = rng.randrange(LEVELS), rng.random() < 0.5
        k1.see(x, level, with_depth=with_depth, descriptor=_flipped(descriptor, flips), node=node1)
    for keyframe in (k1, k2):  # clutter: keypoints of nothing the other sees
        for _ in range(15):
            bearing = _unit([rng.uniform(-0.5, 0.5), rng.uniform(-0.4, 0.4), 1.0])
            level, descriptor = rng.randrange(LEVELS), _random_descriptor(rng)
            keyframe.add(bearing, level, None, descriptor, rng.randrange(7))
    return _case(f"{'sideways' if sideways else 'scene'}-{seed}", k2, k1, [])


def main() -> None:
    """Write the fixture, asserting every rule decides some pair."""
    cases = [hand_built()]
    cases += [scene(seed, sideways=False) for seed in range(4)]
    cases += [scene(seed, sideways=True) for seed in range(4, 6)]
    assert all(DECIDED.values()), f"a rule decides no pair: {DECIDED}"
    assert all(EVENTS.values()), f"an event never happens: {EVENTS}"
    assert all(c["pairs"] for c in cases), "a case matches nothing"
    doc = {
        "description": "Triangulation matches by the rules (generate_triangulation_matches.py).",
        "oracle": "every pair tested by the rules alone, in plain float64 arithmetic",
        "tolerance": {
            "comparison": "exact",
            "rtol": 0.0,
            "atol": 0.0,
            "reason": "keypoint rows; no pair within a factor of 2 of a bound",
        },
        "cases": cases,
    }
    (HERE / "triangulation_matches.json").write_text(json.dumps(doc, indent=1) + "\n")


if __name__ == "__main__":
    main()
