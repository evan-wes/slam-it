"""Generate ``orb.json``, the ORB extractor's parity fixture.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_orb

FAST scores come from an independent brute force: for every pixel, every threshold from 254
down is tried against every arc of nine, by the corner's definition. The quadtree and the
extractor have no independent oracle (they are procedures, not formulas), so their cases are
the Python reference's output, as ``ba.json``'s are: the fixture holds the C++ and Rust cores
to it exactly. Every value is an integer, or a level pixel's full-image coordinate (one
division of exact integers), so the policy is ``exact``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.foundations import orb
from slam_it.foundations.image import pyramid
from tests.fixtures.foundations._layout import dumps

OUT = Path(__file__).with_name("orb.json")
EXACT = {"comparison": "exact", "rtol": 0.0, "atol": 0.0, "reason": "integer arithmetic"}
CIRCLE = (
    (0, 3), (1, 3), (2, 2), (3, 1), (3, 0), (3, -1), (2, -2), (1, -3),
    (0, -3), (-1, -3), (-2, -2), (-3, -1), (-3, 0), (-3, 1), (-2, 2), (-1, 3),
)  # fmt: skip


def _is_corner(ring: list[int], centre: int, t: int) -> bool:
    """Whether nine contiguous ring pixels are all brighter, or all darker, by more than t."""
    for start in range(16):
        arc = [ring[(start + k) % 16] for k in range(9)]
        if all(v > centre + t for v in arc) or all(v < centre - t for v in arc):
            return True
    return False


def _brute_scores(image: list[list[int]]) -> list[list[int]]:
    """Find the largest threshold at which each pixel is a corner (or -1) by trying them all."""
    rows, cols = len(image), len(image[0])
    out = [[-1] * cols for _ in range(rows)]
    for y in range(3, rows - 3):
        for x in range(3, cols - 3):
            ring = [image[y + dy][x + dx] for dx, dy in CIRCLE]
            out[y][x] = next(
                (t for t in range(254, -1, -1) if _is_corner(ring, image[y][x], t)), -1
            )
    return out


def _scene(rng: np.random.Generator, rows: int, cols: int) -> NDArray[np.uint8]:
    """Draw a textured image: blocks of random intensity, smoothed edges, a little noise."""
    blocks = rng.integers(20, 236, (rows // 8 + 1, cols // 8 + 1))
    values = np.kron(blocks, np.ones((8, 8), dtype=np.int64))[:rows, :cols]
    values = values + rng.integers(-6, 7, (rows, cols))
    out: NDArray[np.uint8] = np.clip(values, 0, 255).astype(np.uint8)
    return out


_FIELDS = ("xy", "level", "level_xy", "response", "angle")


def _keypoints(image: NDArray[np.uint8], options: orb.Options) -> dict[str, Any]:
    """Return the reference's keypoints, as lists."""
    found = orb.detect(image, options)
    return {field: getattr(found, field).tolist() for field in _FIELDS}


def _features(image: NDArray[np.uint8], options: orb.Options) -> dict[str, Any]:
    """Return the reference's features, as lists."""
    found = orb.extract(image, options)
    return {
        **{field: getattr(found.keypoints, field).tolist() for field in _FIELDS},
        "descriptors": found.descriptors.tolist(),
        "scale_factor": found.scale_factor,
        "levels": found.levels,
    }


def _distribute_case(
    name: str, xy: NDArray[np.int64], response: NDArray[np.int64], bounds: list[int], target: int
) -> dict[str, Any]:
    """One quadtree case, its expected value from the reference."""
    box = (bounds[0], bounds[1], bounds[2], bounds[3])
    kept = orb.distribute(xy, response, box, target)
    return {
        "name": name,
        "input": [xy.tolist(), response.tolist(), bounds, target],
        "expected": kept.tolist(),
    }


def main() -> None:
    """Write the fixture."""
    rng = np.random.default_rng(11)
    square = np.full((14, 16), 40, dtype=np.uint8)
    square[4:11, 5:12] = 200
    noise = rng.integers(0, 256, (12, 13)).astype(np.uint8)
    blocks = _scene(rng, 16, 18)
    spread = rng.integers(0, 60, (40, 2))
    clustered = np.vstack([rng.integers(0, 8, (30, 2)), rng.integers(40, 60, (5, 2))])
    coincident = np.array([[3, 4], [3, 4], [3, 4], [10, 2]])
    wide = np.column_stack([rng.integers(100, 400, 60), rng.integers(50, 120, 60)])
    distribute_cases = [
        _distribute_case("spread, careful", spread, rng.integers(0, 50, 40), [0, 60, 0, 60], 13),
        _distribute_case("spread, all", spread, rng.integers(0, 50, 40), [0, 60, 0, 60], 100),
        _distribute_case("clustered", clustered, rng.integers(0, 9, 35), [0, 60, 0, 60], 9),
        _distribute_case(
            "coincident corners", coincident, np.array([5, 9, 9, 1]), [0, 16, 0, 8], 4
        ),
        _distribute_case(
            "wide box, five columns", wide, rng.integers(7, 90, 60), [100, 400, 50, 120], 17
        ),
        _distribute_case("target zero", spread, rng.integers(0, 50, 40), [0, 60, 0, 60], 0),
        _distribute_case(
            "no corners",
            np.zeros((0, 2), dtype=np.int64),
            np.zeros(0, dtype=np.int64),
            [0, 5, 0, 5],
            3,
        ),
    ]
    detect_cases = [
        (
            "a 120x160 scene, 3 levels",
            _scene(rng, 120, 160),
            orb.Options(n_features=150, levels=3, scale_factor=1.2),
        ),
        (
            "a 96x200 scene, 4 levels at 1.5",
            _scene(rng, 96, 200),
            orb.Options(n_features=90, levels=4, scale_factor=1.5),
        ),
        (
            "low contrast: the lower threshold only",
            (_scene(rng, 80, 90) // 8 + 90).astype(np.uint8),
            orb.Options(n_features=60, levels=2),
        ),
    ]
    extract_cases = [
        ("a 90x110 scene, 2 levels", _scene(rng, 90, 110), orb.Options(n_features=70, levels=2)),
        (
            "a 100x140 scene, 3 levels at 1.5",
            _scene(rng, 100, 140),
            orb.Options(n_features=80, levels=3, scale_factor=1.5),
        ),
    ]
    functions: dict[str, Any] = {
        "fast_scores": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {"name": name, "input": [image.tolist()], "expected": _brute_scores(image.tolist())}
                for name, image in [
                    ("a bright square", square),
                    ("noise", noise),
                    ("blocks", blocks),
                ]
            ],
        },
        "distribute": {"unpack": True, "tolerance": EXACT, "cases": distribute_cases},
        "detect": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": name,
                    "input": [image.tolist(), dict(vars(options))],
                    "expected": _keypoints(image, options),
                }
                for name, image, options in detect_cases
            ],
        },
        "extract": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": name,
                    "input": [image.tolist(), dict(vars(options))],
                    "expected": _features(image, options),
                }
                for name, image, options in extract_cases
            ],
        },
        "extract_levels": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": f"{name}, from its pyramid",
                    "input": [
                        [
                            level.tolist()
                            for level in pyramid(image, options.levels, options.scale_factor)
                        ],
                        dict(vars(options)),
                    ],
                    "expected": _features(image, options),
                }
                for name, image, options in extract_cases
            ],
        },
    }
    fixture = {
        "description": "ORB features: parity fixture (generate_orb.py).",
        "oracle": "FAST scores by brute force; the rest from the reference",
        "functions": functions,
    }
    OUT.write_text(dumps(fixture))


if __name__ == "__main__":
    main()
