"""Generate ``image.json``, the fixed-point image parity fixture, by direct per-pixel loops.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_image

Independent of the module under test, nothing imported from slam_it_all: each operation is
written from its definition, one output pixel at a time, with exact rationals
(:class:`fractions.Fraction`) where the module uses integer tricks. A blur is the direct 2D
sum over the kernel's outer product, reflecting by stepping back inside the image; a resize
computes each source coordinate ``(d + 1/2) n_in / n_out - 1/2`` as a rational and rounds its
fraction; a remap rounds each coordinate's exact rational value. Every output is exact, so
the policy is ``exact``.
"""

from __future__ import annotations

import math
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np

from tests.fixtures.foundations._layout import dumps

OUT = Path(__file__).with_name("image.json")
GRAY = (9798, 19235, 3735, 15)  # BT.601 over 2^15, OpenCV's cvtColor
RESIZE_BITS = 11
REMAP_BITS = 5
EXACT = {"comparison": "exact", "rtol": 0.0, "atol": 0.0, "reason": "integer arithmetic"}

Grid = list[list[int]]


def _round_half_up(value: Fraction) -> int:
    """Round a rational half up."""
    return math.floor(value + Fraction(1, 2))


def _to_gray(rgb: list[list[list[int]]]) -> Grid:
    """Weighted sum per pixel, rounded half up."""
    r, g, b, bits = GRAY
    return [
        [_round_half_up(Fraction(r * p[0] + g * p[1] + b * p[2], 2**bits)) for p in row]
        for row in rgb
    ]


def _reflect(i: int, n: int) -> int:
    """Step an index back inside ``[0, n)``, reflecting about the edge pixels themselves."""
    if n == 1:
        return 0
    while not 0 <= i < n:
        i = -i if i < 0 else 2 * (n - 1) - i
    return i


def _blur(image: Grid, kernel: list[int]) -> Grid:
    """Sum over the kernel's outer product, divided by its total squared."""
    rows, cols, radius = len(image), len(image[0]), len(kernel) // 2
    total = sum(kernel) ** 2
    out = []
    for y in range(rows):
        row = []
        for x in range(cols):
            acc = 0
            for i, wy in enumerate(kernel):
                for j, wx in enumerate(kernel):
                    source = image[_reflect(y + i - radius, rows)]
                    acc += wy * wx * source[_reflect(x + j - radius, cols)]
            row.append(_round_half_up(Fraction(acc, total)))
        out.append(row)
    return out


def _source(d: int, n_in: int, n_out: int) -> tuple[int, int, int]:
    """Destination index `d`'s two source indices and the second's weight, over 2^11."""
    s = Fraction(2 * d + 1, 2) * n_in / n_out - Fraction(1, 2)
    s = max(s, Fraction(0))
    whole = math.floor(s)
    weight = _round_half_up((s - whole) * 2**RESIZE_BITS)
    if weight == 2**RESIZE_BITS:
        whole, weight = whole + 1, 0
    if whole >= n_in - 1:
        whole, weight = n_in - 1, 0
    return whole, min(whole + 1, n_in - 1), weight


def _resize(image: Grid, rows: int, cols: int) -> Grid:
    """Bilinear per output pixel with the rounded integer weights, rounded once."""
    one = 2**RESIZE_BITS
    out = []
    for y in range(rows):
        y0, y1, fy = _source(y, len(image), rows)
        row = []
        for x in range(cols):
            x0, x1, fx = _source(x, len(image[0]), cols)
            acc = (
                image[y0][x0] * (one - fx) * (one - fy)
                + image[y0][x1] * fx * (one - fy)
                + image[y1][x0] * (one - fx) * fy
                + image[y1][x1] * fx * fy
            )
            row.append(_round_half_up(Fraction(acc, one * one)))
        out.append(row)
    return out


def _pyramid(image: Grid, levels: int, scale_factor: float) -> list[Grid]:
    """Each level resized from the previous to ``round(n / s^l)``, half up."""
    out, scale = [image], 1.0
    for _ in range(1, levels):
        scale *= scale_factor
        rows = math.floor(len(image) / scale + 0.5)
        cols = math.floor(len(image[0]) / scale + 0.5)
        out.append(_resize(out[-1], rows, cols))
    return out


def _remap(image: Grid, coordinates: list[list[list[float]]]) -> Grid:
    """Bilinear at each coordinate rounded to 1/32, zero outside the image."""
    rows, cols, one = len(image), len(image[0]), 2**REMAP_BITS

    def at(r: int, c: int) -> int:
        return image[r][c] if 0 <= r < rows and 0 <= c < cols else 0

    out = []
    for map_row in coordinates:
        row = []
        for x, y in map_row:
            qx = _round_half_up(Fraction(min(max(x, -2.0), cols + 1.0)) * one)
            qy = _round_half_up(Fraction(min(max(y, -2.0), rows + 1.0)) * one)
            ix, fx = qx // one, qx % one
            iy, fy = qy // one, qy % one
            acc = (
                at(iy, ix) * (one - fx) * (one - fy)
                + at(iy, ix + 1) * fx * (one - fy)
                + at(iy + 1, ix) * (one - fx) * fy
                + at(iy + 1, ix + 1) * fx * fy
            )
            row.append(_round_half_up(Fraction(acc, one * one)))
        out.append(row)
    return out


def _smooth(rng: np.random.Generator, rows: int, cols: int) -> Grid:
    """Draw a random image with structure: a gradient, a bright square and noise."""
    y, x = np.mgrid[0:rows, 0:cols]
    values = 40 + 7 * x + 5 * y + rng.integers(0, 30, (rows, cols))
    values[rows // 3 : 2 * rows // 3, cols // 3 : 2 * cols // 3] += 90
    grid: Grid = np.clip(values, 0, 255).astype(int).tolist()
    return grid


def main() -> None:
    """Write the fixture."""
    rng = np.random.default_rng(7)
    noise = rng.integers(0, 256, (9, 11)).tolist()
    scene = _smooth(rng, 12, 16)
    white = [[255] * 7 for _ in range(5)]  # the largest sums a kernel can make
    primaries = [
        [[255, 255, 255], [0, 0, 0], [255, 0, 0]],
        [[0, 255, 0], [0, 0, 255], [128, 128, 128]],
    ]
    rgb = rng.integers(0, 256, (5, 7, 3)).tolist()
    remap_map = [[[x * 1.37 - 1.1, y * 0.83 + 0.4 + 0.05 * x] for x in range(8)] for y in range(6)]
    edges = [
        [[0.0, 0.0], [10.0, 8.0], [10.5, 4.0], [-0.5, 4.0], [5.0, -0.5], [5.0, 8.5]],
        [[1 / 64, 2.0], [-1 / 64, 2.0], [3 + 3 / 64, 1.0], [-1.2, 3.0], [11.7, 3.0], [4.0, 9.9]],
        [[1e6, 2.0], [-1e6, 2.0], [3.0, 1e9], [2.25, -3.0], [0.999, 7.999], [9.96875, 7.03125]],
    ]
    functions: dict[str, Any] = {
        "to_gray": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": "primaries and grey",
                    "input": [primaries],
                    "expected": _to_gray(primaries),
                },
                {"name": "random 5x7", "input": [rgb], "expected": _to_gray(rgb)},
            ],
        },
        "blur": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": f"{name}",
                    "input": [image, kernel],
                    "expected": _blur(image, kernel),
                }
                for name, image, kernel in [
                    ("binomial 5-tap on noise", noise, [1, 4, 6, 4, 1]),
                    ("binomial 7-tap on a scene", scene, [1, 6, 15, 20, 15, 6, 1]),
                    ("7-tap wider than a 2-row image", noise[:2], [1, 6, 15, 20, 15, 6, 1]),
                    ("one pixel", [[200]], [1, 2, 1]),
                    ("identity kernel", noise, [1]),
                    ("uneven weights", scene, [0, 3, 1, 4, 0]),
                    # The vertical sums' widths: 32 bits up to 12-bit kernels, 64 above.
                    ("12-bit kernel on white, the widest 32-bit sums", white, [1, 4094, 1]),
                    ("13-bit kernel on noise, 64-bit sums", noise, [1, 8190, 1]),
                    ("15-bit kernel on white, the widest 64-bit sums", white, [1, 32766, 1]),
                ]
            ],
        },
        "resize": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": f"{len(image)}x{len(image[0])} to {rows}x{cols}",
                    "input": [image, rows, cols],
                    "expected": _resize(image, rows, cols),
                }
                for image, rows, cols in [
                    (scene, 10, 13),
                    (scene, 5, 7),
                    (scene, 25, 33),
                    (scene, 12, 16),
                    (scene, 1, 1),
                    (noise, 3, 11),
                    ([[17]], 3, 4),
                ]
            ],
        },
        "pyramid": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {
                    "name": f"{levels} levels at {scale}",
                    "input": [image, levels, scale],
                    "expected": _pyramid(image, levels, scale),
                }
                for image, levels, scale in [
                    (_smooth(rng, 24, 31), 5, 1.2),
                    (scene, 3, 2.0),
                    (scene, 1, 1.2),
                ]
            ],
        },
        "remap": {
            "unpack": True,
            "tolerance": EXACT,
            "cases": [
                {"name": name, "input": [noise, where], "expected": _remap(noise, where)}
                for name, where in [
                    ("a smooth warp", remap_map),
                    ("edges, ties and far outside", edges),
                ]
            ],
        },
    }
    fixture = {
        "description": "Fixed-point image operations: parity fixture (generate_image.py).",
        "oracle": "per-pixel loops from each definition, exact rationals",
        "functions": functions,
    }
    OUT.write_text(dumps(fixture))


if __name__ == "__main__":
    main()
