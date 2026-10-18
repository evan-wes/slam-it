"""A dashcam's config: where its recordings and the steps' outputs live, and what was measured.

One TOML file, kept with the recordings rather than in the repo: it names the card's copy, the
folder the steps write into, the rig's measured baseline, and per camera its mask's drawn parts,
its mask file, and its lens once calibrated. ``docs/dashcam.template.toml`` is the template,
every key explained. Relative paths are taken from the config file's folder. An unknown key is
refused, so a misspelt one is not silently ignored.
"""

from __future__ import annotations

import math
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from slam_it.eval import lenses, video
from slam_it.eval.dashcam import ingest
from slam_it.eval.dashcam.mask import Line
from slam_it.foundations.errors import InvalidInputError


@dataclass(frozen=True)
class Camera:
    """One camera's settings.

    Attributes
    ----------
    mask : pathlib.Path
        Its static mask: written by ``mask``, read by ``calibrate`` and ``import``.
    bottom : float or None
        Drop everything below this fraction of the height too.
    lines : tuple of Line
        Lines drawn into the mask.
    lens : slam_it.eval.lenses.Pinhole or Fisheye, or None
        Its lens at the video's size, once calibrated.
    """

    mask: Path
    bottom: float | None
    lines: tuple[Line, ...]
    lens: lenses.Pinhole | lenses.Fisheye | None


@dataclass(frozen=True)
class Config:
    """A dashcam's config (the module docstring).

    Attributes
    ----------
    root : pathlib.Path
        The card's root, or its copy: it holds ``video/F`` and ``video/R``.
    work : pathlib.Path
        The folder the steps write into, a folder a drive.
    baseline : float
        The measured distance from the front camera to the rear, in metres.
    front, rear : Camera
        Each camera's settings.
    limits : slam_it.eval.video.Limits
        The import's limits.
    """

    root: Path
    work: Path
    baseline: float
    front: Camera
    rear: Camera
    limits: video.Limits

    def camera(self, name: str) -> Camera:
        """Return a camera's settings, ``"F"`` or ``"R"``."""
        if name not in ("F", "R"):
            raise InvalidInputError(f"camera: expected F or R, got {name!r}")
        return self.front if name == "F" else self.rear


def _table(data: Mapping[str, Any], name: str, keys: set[str]) -> Mapping[str, Any]:
    """Return a sub-table, refusing one missing (if required) or holding an unknown key."""
    table = data.get(name, {})
    if not isinstance(table, Mapping):
        raise InvalidInputError(f"[{name}]: expected a table")
    unknown = set(table) - keys
    if unknown:
        raise InvalidInputError(f"[{name}]: unknown keys {sorted(unknown)}")
    return table


def _number(table: Mapping[str, Any], key: str, where: str) -> float:
    """Return a finite number from a table."""
    value = table.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise InvalidInputError(f"[{where}] {key}: expected a number, got {value!r}")
    return float(value)


def _path(table: Mapping[str, Any], key: str, where: str, base: Path) -> Path:
    """Return a path from a table, taken from `base` if relative."""
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise InvalidInputError(f"[{where}] {key}: expected a path")
    return base / Path(value).expanduser()


def _lens(table: Mapping[str, Any], where: str) -> lenses.Pinhole | lenses.Fisheye:
    """Return a lens table's lens: pinhole or fisheye, at its size."""
    model = table.get("model")
    size = table.get("size")
    if not (isinstance(size, list) and len(size) == 2 and all(isinstance(v, int) for v in size)):
        raise InvalidInputError(f"[{where}] size: expected [width, height]")
    fx, fy, cx, cy = (_number(table, key, where) for key in ("fx", "fy", "cx", "cy"))
    camera = lenses.Pinhole(fx, fy, cx, cy, size[0], size[1])
    if model == "pinhole":
        return camera
    if model == "fisheye":
        k = table.get("k")
        if not (isinstance(k, list) and len(k) == 4):
            raise InvalidInputError(f"[{where}] k: expected 4 coefficients")
        return lenses.Fisheye(camera, (k[0], k[1], k[2], k[3]))
    raise InvalidInputError(f"[{where}] model: expected 'pinhole' or 'fisheye', got {model!r}")


def _camera(data: Mapping[str, Any], name: str, base: Path) -> Camera:
    """Return a camera's table's settings."""
    table = _table(data, name, {"mask", "bottom", "lines", "lens"})
    bottom = None if "bottom" not in table else _number(table, "bottom", name)
    lines = []
    for line in table.get("lines", []):
        if not (isinstance(line, list) and len(line) == 5):
            raise InvalidInputError(f"[{name}] lines: expected [x0, y0, x1, y1, width] each")
        lines.append(Line(*(float(v) for v in line)))
    lens = None
    if "lens" in table:
        keys = {"model", "size", "fx", "fy", "cx", "cy", "k"}
        lens = _lens(_table(table, "lens", keys), f"{name}.lens")
    return Camera(_path(table, "mask", name, base), bottom, tuple(lines), lens)


def read(path: str | Path) -> Config:
    """Read a config file (the module docstring).

    Parameters
    ----------
    path : str or pathlib.Path
        The TOML file.

    Returns
    -------
    Config
        Its settings.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not TOML, a key is unknown or missing, or a value is out of range.
    """
    file = Path(path)
    try:
        data = tomllib.loads(file.read_text(encoding="utf-8"))
    except tomllib.TOMLDecodeError as error:
        raise InvalidInputError(f"{file.name}: not TOML ({error})") from None
    unknown = set(data) - {"card", "work", "rig", "front", "rear", "import"}
    if unknown:
        raise InvalidInputError(f"{file.name}: unknown tables {sorted(unknown)}")
    base = file.resolve().parent
    root = _path(_table(data, "card", {"root"}), "root", "card", base)
    work = _path(_table(data, "work", {"folder"}), "folder", "work", base)
    baseline = _number(_table(data, "rig", {"baseline"}), "baseline", "rig")
    if not baseline > 0.0:
        raise InvalidInputError(f"[rig] baseline: expected a positive distance, got {baseline}")
    limits = _table(data, "import", {"max_seconds", "max_frames", "short_side"})
    taken = video.Limits(
        max_seconds=float(limits.get("max_seconds", ingest.LIMITS.max_seconds)),
        max_frames=int(limits.get("max_frames", ingest.LIMITS.max_frames)),
        short_side=int(limits.get("short_side", ingest.LIMITS.short_side)),
    )
    return Config(
        root, work, baseline, _camera(data, "front", base), _camera(data, "rear", base), taken
    )
