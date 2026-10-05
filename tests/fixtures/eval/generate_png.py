"""Generate the PNG fixtures in ``png/`` and their expected values, ``png.json``.

Run from the repository root after changing the cases (the fixtures are committed)::

    python -m tests.fixtures.eval.generate_png

Pillow writes each PNG from a known array; the expected values follow from that array by the
readers' definitions, written out here: gray as stored, colour through BT.601's 15-bit
weights rounded half up (OpenCV's), a palette through its table, a 1-bit image as 0 and 255,
16-bit values as stored. The Python, C++ and Rust readers are held to them exactly.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from PIL import Image

HERE = Path(__file__).parent
FOLDER = HERE / "png"


def _gray(rgb: NDArray[np.int64]) -> list[list[int]]:
    """Convert to gray: BT.601 in 15 bits, rounded half up."""
    total = 9798 * rgb[..., 0] + 19235 * rgb[..., 1] + 3735 * rgb[..., 2]
    rounded: list[list[int]] = ((total + (1 << 14)) >> 15).tolist()
    return rounded


def main() -> None:
    """Write the PNGs and the expected values."""
    rng = np.random.default_rng(5)
    FOLDER.mkdir(exist_ok=True)
    cases: dict[str, dict[str, Any]] = {}

    gray = rng.integers(0, 256, (7, 9)).astype(np.uint8)
    Image.fromarray(gray, mode="L").save(FOLDER / "gray.png")
    cases["gray.png"] = {"gray": gray.tolist(), "rgb": np.repeat(gray[..., None], 3, 2).tolist()}

    gray_alpha = rng.integers(0, 256, (5, 6, 2)).astype(np.uint8)
    Image.fromarray(gray_alpha, mode="LA").save(FOLDER / "gray_alpha.png")
    g = gray_alpha[..., 0]
    cases["gray_alpha.png"] = {"gray": g.tolist(), "rgb": np.repeat(g[..., None], 3, 2).tolist()}

    rgb = rng.integers(0, 256, (6, 8, 3)).astype(np.uint8)
    Image.fromarray(rgb, mode="RGB").save(FOLDER / "rgb.png")
    cases["rgb.png"] = {"gray": _gray(rgb.astype(np.int64)), "rgb": rgb.tolist()}

    rgba = rng.integers(0, 256, (4, 5, 4)).astype(np.uint8)
    Image.fromarray(rgba, mode="RGBA").save(FOLDER / "rgba.png")
    cases["rgba.png"] = {
        "gray": _gray(rgba[..., :3].astype(np.int64)),
        "rgb": rgba[..., :3].tolist(),
    }

    table = rng.integers(0, 256, (16, 3)).astype(np.uint8)
    indices = rng.integers(0, 16, (5, 7)).astype(np.uint8)
    palette = Image.fromarray(indices, mode="P")
    palette.putpalette(table.ravel().tolist())
    palette.save(FOLDER / "palette.png")
    expanded = table[indices]
    cases["palette.png"] = {"gray": _gray(expanded.astype(np.int64)), "rgb": expanded.tolist()}

    widened = (rng.integers(0, 2, (6, 10)) * 255).astype(np.uint8)
    # From 0 and 255 without dithering: fromarray(mode="1") packs a bool array's bits wrongly.
    Image.fromarray(widened, mode="L").convert("1", dither=Image.Dither.NONE).save(
        FOLDER / "bilevel.png"
    )
    cases["bilevel.png"] = {
        "gray": widened.tolist(),
        "rgb": np.repeat(widened[..., None], 3, 2).tolist(),
    }

    depth = rng.integers(0, 65536, (6, 9)).astype(np.uint16)
    depth[0, :3] = [0, 1, 65535]
    Image.fromarray(depth).save(FOLDER / "depth16.png")
    cases["depth16.png"] = {"depth": depth.tolist()}

    (HERE / "png.json").write_text(
        json.dumps({"description": "PNG readers' fixtures (generate_png.py).", "cases": cases})
        + "\n"
    )


if __name__ == "__main__":
    main()
