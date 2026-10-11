"""A phone video's intrinsics: the best source wins; a fisheye lens is undistorted to pinhole.

The sources, tried in order (docs/design/2026-10-09-run-viewer.md, section 5.2), a Stray Scanner
recording's exact camera matrix aside (:mod:`slam_it_all.eval.video` reads it):

1. **A Gyroflow lens profile** (``"profile"``, :func:`match`), matched from the video's
   metadata: OpenCV's fisheye model, which the import undistorts.
2. **The 35mm-equivalent focal length** in the metadata (``"35mm"``, :func:`from_35mm`): a
   pinhole, its centre in the middle, without distortion.
3. **A nominal field of view** (``"nominal"``, :func:`nominal`): the median of the database's
   main-camera profiles (:data:`NOMINAL_DIAGONAL_FOV`).

**The database** is Gyroflow's ``lens_profiles`` (https://github.com/gyroflow/lens_profiles,
CC0), its phones' profiles only, fetched at :data:`PROFILES_COMMIT` by :func:`fetch` (the setup
script and the worker's image run ``python -m slam_it_all.eval.lenses fetch``) into
:func:`default_folder`, never committed. Each profile is a camera matrix and four fisheye
coefficients for one phone, lens and video mode, with its calibration error.

**Matching** (:func:`match`). The profiles are contributed by hand, so their names are free text:
- a profile is usable if it is OpenCV's fisheye model, unstretched and uncropped, calibrated
  within :data:`MAX_RMS` pixels, its model holding out to its image's corners (1,047 of the
  1,164 phone profiles at :data:`PROFILES_COMMIT`);
- its brand and model must equal the video's, compared without case or repeated spaces, but
  whole: an "iPhone 14 Pro Max" never stands in for an "iPhone 14 Pro";
- its lens must be the video's (:func:`lens_kind`: main, ultrawide, telephoto or front), the
  main camera when the video does not say;
- its frame must have the video's aspect within :data:`ASPECT_TOLERANCE`, in the same
  orientation;
- the nearest resolution wins, then the nearest frame rate, then the smallest calibration
  error. The profile is scaled to the video's size.

**Undistortion** (:func:`undistortion`). OpenCV's fisheye model (Kannala-Brandt) bends a ray at
angle theta from the axis to theta_d = theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4
theta^8). Each pixel of the pinhole output maps straight to its source pixel through it, with
no inversion; the output keeps the lens's focal lengths and is the largest rectangle with no
empty border.

**Assumptions the test clips check:** that an Android video names its phone in
``com.android.manufacturer`` and ``com.android.model``; that the 35mm-equivalent focal length is
taken across the frame's diagonal (the usual definition); and how the lenses are named, in
Apple's metadata and in the profiles.

Run as a program::

    python -m slam_it_all.eval.lenses fetch [--into DIR]
    python -m slam_it_all.eval.lenses nominal [--profiles DIR]
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import re
import shutil
import sys
import tarfile
import urllib.request
import uuid
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it_all.foundations.errors import InvalidInputError

PROFILES_COMMIT = "886fc616c9c2c788856f54c21d451b9cb7fc4f44"
"""The commit of Gyroflow's ``lens_profiles`` the profiles are fetched at (2026-07-28)."""

PROFILES_ARCHIVE = f"https://codeload.github.com/gyroflow/lens_profiles/tar.gz/{PROFILES_COMMIT}"
"""The archive :func:`fetch` downloads."""

PHONES = "Mobile phones"
"""The database's folder of phone profiles: the only one fetched."""

MAX_ARCHIVE_BYTES = 64 * 2**20
"""The most :func:`fetch` downloads (the archive was 5 MB at :data:`PROFILES_COMMIT`)."""

MAX_PROFILE_BYTES = 64 * 2**10
"""The largest profile :func:`fetch` keeps (one is about 1.4 KB)."""

MAX_RMS = 2.0
"""The largest calibration error a usable profile has, in pixels at its calibration size."""

ASPECT_TOLERANCE = 0.02
"""How far a profile's aspect (long side over short) may be from the video's."""

FULL_FRAME_DIAGONAL = math.hypot(36.0, 24.0)
"""A 35 mm film frame's diagonal, in millimetres: what a 35mm-equivalent focal length is to."""

NOMINAL_DIAGONAL_FOV = 71.66
"""The median diagonal field of view of the database's usable main-camera profiles, in degrees:
766 of them at :data:`PROFILES_COMMIT` (``python -m slam_it_all.eval.lenses nominal``)."""

_ANGLES = 1 << 14
"""The angles a fisheye's bend is sampled at, over a quarter turn, to find where it stops
rising."""

SOURCES = ("profile", "35mm", "nominal")
"""The intrinsics' sources a video's import can use, in the order tried."""

APPLE_MAKE = "com.apple.quicktime.make"
APPLE_MODEL = "com.apple.quicktime.model"
APPLE_LENS = "com.apple.quicktime.camera.lens_model"
APPLE_FOCAL_35MM = "com.apple.quicktime.camera.focal_length.35mm_equivalent"
ANDROID_MAKE = "com.android.manufacturer"
ANDROID_MODEL = "com.android.model"

METADATA_KEYS = (APPLE_MAKE, APPLE_MODEL, APPLE_LENS, APPLE_FOCAL_35MM, ANDROID_MAKE, ANDROID_MODEL)
"""The metadata fields the intrinsics are chosen from: the only ones an import copies."""

_TELEPHOTO_ZOOM = re.compile(r"\b([2-9]|[1-9]\d)(?:[.,]\d+)?x\b")
"""A zoom of two or more, as "3x": a telephoto."""

_WIDER = re.compile(r"(?<!\d)0?[.,][4-8](?!\d)|\buw\b")
"""A zoom under one, as "0.5", "0,5" or ".6", or "UW": an ultrawide (never inside a number,
as in "29.54fps" or "0,61")."""

_FOCAL_MM = re.compile(r"(?<![\d.,])(\d{1,3})\s*mm\b")
"""A whole number of millimetres, as "24mm" (never the end of a decimal, as in "6.86mm")."""


@dataclass(frozen=True)
class Pinhole:
    """A pinhole camera's intrinsics at an image size, in pixels.

    Pixel centres are at integers, the top-left one at (0, 0).

    Attributes
    ----------
    fx, fy : float
        The focal lengths.
    cx, cy : float
        The principal point.
    width, height : int
        The image size.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        """Hold plain floats and ints, whatever numbers were given; refuse any not finite."""
        for name in ("fx", "fy", "cx", "cy"):
            object.__setattr__(self, name, float(getattr(self, name)))
        for name in ("width", "height"):
            object.__setattr__(self, name, int(getattr(self, name)))
        values = (self.fx, self.fy, self.cx, self.cy)
        if not (all(np.isfinite(values)) and self.fx > 0.0 and self.fy > 0.0):
            raise InvalidInputError(f"intrinsics: expected finite, positive focals, got {values}")
        if not (self.width > 0 and self.height > 0):
            raise InvalidInputError(f"size: expected positive, got {self.width}x{self.height}")

    def turned(self, quarter_turns: int) -> Pinhole:
        """Return the camera of the image turned counter-clockwise by quarter turns.

        Parameters
        ----------
        quarter_turns : int
            How many quarter turns, as ``numpy.rot90`` turns an array.

        Returns
        -------
        Pinhole
            The turned image's camera.
        """
        camera = self
        for _ in range(quarter_turns % 4):
            # numpy.rot90 moves pixel (x, y) of a W-wide image to (y, W - 1 - x).
            camera = Pinhole(
                camera.fy, camera.fx, camera.cy, camera.width - 1 - camera.cx,
                camera.height, camera.width,
            )  # fmt: skip
        return camera

    def scaled(self, width: int, height: int) -> Pinhole:
        """Return the camera of the image resampled to another size.

        Parameters
        ----------
        width, height : int
            The new size.

        Returns
        -------
        Pinhole
            The resampled image's camera, pixel centres kept at integers.
        """
        sx, sy = width / self.width, height / self.height
        return Pinhole(
            self.fx * sx, self.fy * sy, (self.cx + 0.5) * sx - 0.5, (self.cy + 0.5) * sy - 0.5,
            width, height,
        )  # fmt: skip


@dataclass(frozen=True)
class Fisheye:
    """OpenCV's fisheye camera model (Kannala-Brandt).

    Attributes
    ----------
    camera : Pinhole
        Its camera matrix and image size.
    k : tuple of float
        The coefficients k1 to k4 of theta_d = theta (1 + k1 theta^2 + ... + k4 theta^8).
    """

    camera: Pinhole
    k: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        """Hold four plain floats; refuse any not finite."""
        k = tuple(float(v) for v in self.k)
        if len(k) != 4 or not all(np.isfinite(k)):
            raise InvalidInputError(f"k: expected 4 finite coefficients, got {self.k}")
        object.__setattr__(self, "k", k)

    def scaled(self, width: int, height: int) -> Fisheye:
        """Return the model of the image resampled to another size (the angles unchanged)."""
        return Fisheye(self.camera.scaled(width, height), self.k)

    def turned(self, quarter_turns: int) -> Fisheye:
        """Return the model of the image turned counter-clockwise (the model is radial)."""
        return Fisheye(self.camera.turned(quarter_turns), self.k)

    def bend(self, theta: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return the distorted angle theta_d of each ray's angle from the axis."""
        t2 = theta * theta
        k1, k2, k3, k4 = self.k
        out: NDArray[np.float64] = theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
        return out

    def rising(self) -> float:
        """Return the angle up to which the bend rises, and so inverts (at most a quarter turn).

        Found on a grid of :data:`_ANGLES` angles.
        """
        theta = np.linspace(0.0, math.pi / 2, _ANGLES + 1)[1:-1]
        k1, k2, k3, k4 = self.k
        t2 = theta * theta
        slope = 1.0 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
        falling = np.flatnonzero(slope <= 0.0)
        return float(theta[falling[0] - 1]) if falling.size else float(theta[-1])

    def unbend(self, theta_d: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return each ray's angle from the axis for its distorted angle, by bisection.

        The bend rises up to :meth:`rising`, so each distorted angle up to the bend there has
        one angle; bisection cannot step past it, as Newton's method can near its top.

        Raises
        ------
        slam_it_all.foundations.errors.InvalidInputError
            If a distorted angle is negative, or beyond where the bend stops rising: the
            model does not hold out to that pixel.
        """
        target = np.asarray(theta_d, dtype=np.float64)
        top = self.rising()
        if np.any(target < 0.0) or np.any(target > self.bend(np.array(top))):
            raise InvalidInputError("fisheye: the model does not hold out to the image's edge")
        lo, hi = np.zeros_like(target), np.full_like(target, top)
        for _ in range(64):  # halves an interval under two to below 1e-18
            mid = 0.5 * (lo + hi)
            below = self.bend(mid) < target
            lo, hi = np.where(below, mid, lo), np.where(below, hi, mid)
        out: NDArray[np.float64] = 0.5 * (lo + hi)
        return out

    def holds(self) -> bool:
        """Return whether the model inverts out to its image's farthest pixel centre.

        Undistorting the whole image needs it; some calibrations' polynomials turn back before.
        """
        c = self.camera
        corners = [
            ((x - c.cx) / c.fx, (y - c.cy) / c.fy)
            for x in (0, c.width - 1)
            for y in (0, c.height - 1)
        ]
        farthest = max(math.hypot(a, b) for a, b in corners)
        return farthest <= float(self.bend(np.array(self.rising())))

    def source(self, a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return the source pixels of rays given by their pinhole coordinates (x/z, y/z).

        Parameters
        ----------
        a, b : ndarray
            Each ray's normalised pinhole coordinates.

        Returns
        -------
        ndarray, shape (..., 2)
            Each ray's pixel ``(x, y)`` in this camera's image.
        """
        r = np.hypot(a, b)
        theta_d = self.bend(np.arctan(r))
        scale = np.divide(theta_d, r, out=np.ones_like(r), where=r > 0.0)
        c = self.camera
        return np.stack([c.fx * a * scale + c.cx, c.fy * b * scale + c.cy], axis=-1)

    def pinhole(self, x: NDArray[np.float64], y: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return the normalised pinhole coordinates of rays given by their source pixels."""
        c = self.camera
        a_d, b_d = (x - c.cx) / c.fx, (y - c.cy) / c.fy
        theta_d = np.hypot(a_d, b_d)
        theta = self.unbend(theta_d)
        scale = np.divide(np.tan(theta), theta_d, out=np.ones_like(theta_d), where=theta_d > 0.0)
        return np.stack([a_d * scale, b_d * scale], axis=-1)


def undistortion(lens: Fisheye) -> tuple[Pinhole, NDArray[np.float64]]:
    """Return the pinhole a fisheye image undistorts to, and each output pixel's source pixel.

    The output keeps the lens's focal lengths and is the largest rectangle whose every pixel
    samples between the source's outermost pixel centres, so that bilinear sampling never
    reads past the image: its edges are where those centres are nearest the middle.

    Parameters
    ----------
    lens : Fisheye
        The source image's model.

    Returns
    -------
    tuple
        The output's camera, and its map, shape ``(height, width, 2)``: each output pixel's
        source ``(x, y)``, for :func:`slam_it_all.foundations.image.remap`.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the model does not invert over the image, or leaves no rectangle.
    """
    c = lens.camera
    count = 4 * max(c.width, c.height)
    last_x, last_y = c.width - 1.0, c.height - 1.0
    xs, ys = np.linspace(0.0, last_x, count), np.linspace(0.0, last_y, count)
    left = lens.pinhole(np.zeros(count), ys)[:, 0].max()
    right = lens.pinhole(np.full(count, last_x), ys)[:, 0].min()
    top = lens.pinhole(xs, np.zeros(count))[:, 1].max()
    bottom = lens.pinhole(xs, np.full(count, last_y))[:, 1].min()
    # Pixel x of the output is (x - cx) / fx: from `left` at 0 to at most `right` at width - 1.
    width = math.floor(c.fx * (right - left)) + 1
    height = math.floor(c.fy * (bottom - top)) + 1
    if right <= left or bottom <= top:
        raise InvalidInputError("fisheye: no rectangle without an empty border")
    out = Pinhole(c.fx, c.fy, -c.fx * left, -c.fy * top, width, height)
    gx, gy = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    mapping = lens.source((gx - out.cx) / out.fx, (gy - out.cy) / out.fy)
    # The edges were sampled, not solved: a pixel may land a hair (1e-5) past a centre.
    np.clip(mapping[..., 0], 0.0, last_x, out=mapping[..., 0])
    np.clip(mapping[..., 1], 0.0, last_y, out=mapping[..., 1])
    return out, mapping


@dataclass(frozen=True)
class Profile:
    """A usable Gyroflow lens profile.

    Attributes
    ----------
    name : str
        Its file's name, without ``.json``.
    brand, model : str
        The phone, as the profile names it.
    lens : str
        The lens's kind (:func:`lens_kind`).
    fps : float
        The frame rate it was calibrated at.
    rms : float
        Its calibration error, in pixels at its calibration size.
    fisheye : Fisheye
        The model, at its calibration size.
    """

    name: str
    brand: str
    model: str
    lens: str
    fps: float
    rms: float
    fisheye: Fisheye


def _normal(text: str) -> str:
    """Return a name without case or repeated spaces."""
    return " ".join(text.lower().split())


def lens_kind(text: str, *, equivalent: bool = True) -> str:
    """Return the kind of lens a free-text name says: front, ultrawide, telephoto or main.

    Parameters
    ----------
    text : str
        A profile's lens, setting and name ("0,5 Wide", "3x", "13mm"), or Apple's lens model
        ("iPhone 14 Pro back ultra wide camera 2.22mm f/2.2").
    equivalent : bool, optional
        Whether a number of millimetres in it is a 35mm-equivalent focal length, as a
        profile's are (16 mm and under an ultrawide, 50 mm and over a telephoto). Apple's
        lens model gives the physical focal length, so it is read by its words alone.

    Returns
    -------
    str
        ``"front"``, ``"ultrawide"``, ``"telephoto"`` or ``"main"`` (the default).
    """
    words = _normal(text)
    if "front" in words or "selfie" in words:
        return "front"
    if "ultra" in words or _WIDER.search(words):
        return "ultrawide"
    if "tele" in words or _TELEPHOTO_ZOOM.search(words):
        return "telephoto"
    for focal in (int(m) for m in _FOCAL_MM.findall(words)) if equivalent else ():
        if focal <= 16:
            return "ultrawide"
        if focal >= 50:
            return "telephoto"
    return "main"


def _number(value: Any, name: str) -> float:  # noqa: ANN401 -- a JSON value
    """Return a JSON number as a float, refusing anything else."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise InvalidInputError(f"{name}: expected a number, got {value!r}")
    return float(value)


def _usable(data: object) -> bool:
    """Whether a profile's JSON is OpenCV's fisheye model, unstretched and uncropped."""
    if not isinstance(data, dict) or data.get("crop") is not None:
        return False
    if data.get("distortion_model") not in (None, "opencv_fisheye"):
        return False
    stretches = ("input_horizontal_stretch", "input_vertical_stretch")
    return all(_number(data.get(key, 1.0), key) == 1.0 for key in stretches)


def read_profile(path: str | Path) -> Profile | None:
    """Read a Gyroflow lens profile; None if it is not usable (the module docstring).

    Parameters
    ----------
    path : str or pathlib.Path
        The profile's file.

    Returns
    -------
    Profile or None
        The profile, or None if it is another model, stretched, cropped, malformed or
        calibrated worse than :data:`MAX_RMS`, or its model does not hold out to its image's
        corners (:meth:`Fisheye.holds`).
    """
    file = Path(path)
    try:
        data = json.loads(file.read_text(encoding="utf-8"))
        if not _usable(data):
            return None
        params, size = data["fisheye_params"], data["calib_dimension"]
        matrix = np.array(params["camera_matrix"], dtype=np.float64)
        rms = _number(params["RMS_error"], "RMS_error")
        coefficients = [_number(v, "k") for v in params["distortion_coeffs"]]
        if matrix.shape != (3, 3) or matrix[0, 1] != 0.0 or rms > MAX_RMS or len(coefficients) != 4:
            return None
        width, height = int(_number(size["w"], "w")), int(_number(size["h"], "h"))
        camera = Pinhole(matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2], width, height)
        k1, k2, k3, k4 = coefficients
        fisheye = Fisheye(camera, (k1, k2, k3, k4))
        if not fisheye.holds():
            return None
        texts = " ".join(str(data.get(key) or "") for key in ("lens_model", "camera_setting"))
        return Profile(
            file.stem,
            str(data["camera_brand"]),
            str(data["camera_model"]),
            lens_kind(f"{texts} {file.stem}"),
            _number(data.get("fps", 0.0), "fps"),
            rms,
            fisheye,
        )
    except (KeyError, TypeError, ValueError, InvalidInputError, json.JSONDecodeError):
        return None


def load(folder: str | Path) -> list[Profile]:
    """Read every usable profile in a folder (its ``.json`` files, at any depth).

    Parameters
    ----------
    folder : str or pathlib.Path
        The profiles' folder.

    Returns
    -------
    list of Profile
        The usable ones, by name.
    """
    found = (read_profile(path) for path in sorted(Path(folder).rglob("*.json")))
    return [profile for profile in found if profile is not None]


@dataclass(frozen=True)
class Video:
    """What the intrinsics are chosen from: a video's metadata, stored size and frame rate.

    Attributes
    ----------
    metadata : dict of str to str
        Its metadata fields of :data:`METADATA_KEYS`.
    width, height : int
        Its frame size, as stored (before any declared rotation).
    fps : float or None
        Its frame rate, if known.
    """

    metadata: dict[str, str]
    width: int
    height: int
    fps: float | None = None


def _aspect(width: int, height: int) -> float:
    """Return an image's long side over its short one."""
    return max(width, height) / min(width, height)


def match(profiles: Sequence[Profile], video: Video) -> tuple[Profile, Fisheye] | None:
    """Return the profile that best fits a video (the module docstring), scaled to it.

    Parameters
    ----------
    profiles : sequence of Profile
        The usable profiles.
    video : Video
        The video.

    Returns
    -------
    tuple of (Profile, Fisheye) or None
        The profile, and its model scaled to the video's size; None if none fits.
    """
    metadata, width, height, fps = video.metadata, video.width, video.height, video.fps
    brand = _normal(metadata.get(APPLE_MAKE) or metadata.get(ANDROID_MAKE) or "")
    model = _normal(metadata.get(APPLE_MODEL) or metadata.get(ANDROID_MODEL) or "")
    if not (brand and model):
        return None
    lens = lens_kind(metadata.get(APPLE_LENS, ""), equivalent=False)
    landscape = width >= height
    candidates = [
        p
        for p in profiles
        if (_normal(p.brand), _normal(p.model), p.lens) == (brand, model, lens)
        and (p.fisheye.camera.width >= p.fisheye.camera.height) == landscape
        and abs(_aspect(p.fisheye.camera.width, p.fisheye.camera.height) - _aspect(width, height))
        <= ASPECT_TOLERANCE
    ]
    if not candidates:
        return None
    longest = max(width, height)

    def rank(p: Profile) -> tuple[float, float, float]:
        size = abs(math.log(max(p.fisheye.camera.width, p.fisheye.camera.height) / longest))
        return (size, abs(p.fps - fps) if fps else 0.0, p.rms)

    best = min(candidates, key=rank)
    return best, best.fisheye.scaled(width, height)


def from_35mm(focal_35mm: float, width: int, height: int) -> Pinhole:
    """Return a pinhole from a 35mm-equivalent focal length, taken across the diagonal.

    Parameters
    ----------
    focal_35mm : float
        The focal length, in millimetres of a 36 by 24 mm frame.
    width, height : int
        The video's frame size.

    Returns
    -------
    Pinhole
        Its focal length scaled from the frame's diagonal, its centre in the middle.
    """
    focal = focal_35mm * math.hypot(width, height) / FULL_FRAME_DIAGONAL
    return Pinhole(focal, focal, (width - 1) / 2, (height - 1) / 2, width, height)


def nominal(width: int, height: int) -> Pinhole:
    """Return a pinhole of :data:`NOMINAL_DIAGONAL_FOV`, its centre in the middle.

    Parameters
    ----------
    width, height : int
        The video's frame size.

    Returns
    -------
    Pinhole
        The camera.
    """
    half = math.radians(NOMINAL_DIAGONAL_FOV) / 2
    focal = math.hypot(width, height) / 2 / math.tan(half)
    return Pinhole(focal, focal, (width - 1) / 2, (height - 1) / 2, width, height)


@dataclass(frozen=True)
class Chosen:
    """A video's intrinsics, and where they came from.

    Attributes
    ----------
    lens : Pinhole or Fisheye
        The camera, for the frames as stored, at the video's size.
    source : str
        One of :data:`SOURCES`, or ``"given"`` for intrinsics the caller gives.
    profile : str or None
        The profile's name, if the source is a profile.
    """

    lens: Pinhole | Fisheye
    source: str
    profile: str | None = None


def choose(video: Video, profiles: Sequence[Profile], source: str = "auto") -> Chosen:
    """Return a video's intrinsics from the best source that has them, or the one asked for.

    Parameters
    ----------
    video : Video
        The video.
    profiles : sequence of Profile
        The usable profiles (none, without the database).
    source : str, optional
        ``"auto"`` for the first of :data:`SOURCES` that has intrinsics, or one of them.

    Returns
    -------
    Chosen
        The intrinsics and their source.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the source is unknown, or the one asked for has no intrinsics for the video.
    """
    if source != "auto" and source not in SOURCES:
        raise InvalidInputError(f"source: expected auto or one of {SOURCES}, got {source!r}")
    if source in ("auto", "profile"):
        found = match(profiles, video)
        if found is not None:
            return Chosen(found[1], "profile", found[0].name)
        if source == "profile":
            raise InvalidInputError("source: no lens profile matches the video")
    if source in ("auto", "35mm"):
        try:
            focal = float(video.metadata.get(APPLE_FOCAL_35MM, "nan"))
        except ValueError:
            focal = math.nan
        if math.isfinite(focal) and focal > 0.0:
            return Chosen(from_35mm(focal, video.width, video.height), "35mm")
        if source == "35mm":
            raise InvalidInputError("source: the video has no 35mm-equivalent focal length")
    return Chosen(nominal(video.width, video.height), "nominal")


def nominal_fov(profiles: Sequence[Profile]) -> tuple[float, int]:
    """Return the median diagonal field of view of main-camera profiles, and how many.

    Parameters
    ----------
    profiles : sequence of Profile
        The usable profiles.

    Returns
    -------
    tuple of (float, int)
        The median, in degrees, and the number of main-camera profiles it is over.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If no profile is of a main camera.
    """
    angles = []
    for p in profiles:
        if p.lens != "main":
            continue
        c = p.fisheye.camera
        corner = np.array([math.hypot((c.width / 2) / c.fx, (c.height / 2) / c.fy)])
        angles.append(2 * math.degrees(float(p.fisheye.unbend(corner)[0])))
    if not angles:
        raise InvalidInputError("profiles: none of a main camera")
    return float(np.median(angles)), len(angles)


def default_folder() -> Path:
    """Return where the profiles are: ``$SLAM_IT_ALL_LENS_PROFILES``, else the user's cache.

    Returns
    -------
    pathlib.Path
        The folder: ``~/.cache/slam-it-all/lens_profiles/<commit>`` by default.
    """
    given = os.environ.get("SLAM_IT_ALL_LENS_PROFILES")
    if given:
        return Path(given)
    return Path.home() / ".cache" / "slam-it-all" / "lens_profiles" / PROFILES_COMMIT


def _keep_phones(archive: tarfile.TarFile, folder: Path) -> int:
    """Copy an archive's phone profiles into a folder under names made from their paths."""
    kept = 0
    for member in archive.getmembers():
        parts = PurePosixPath(member.name).parts
        if not member.isfile() or len(parts) < 3 or parts[1] != PHONES:
            continue
        if ".." in parts or not member.name.endswith(".json") or member.size > MAX_PROFILE_BYTES:
            continue
        body = archive.extractfile(member)
        if body is not None:
            (folder / "__".join(parts[2:])).write_bytes(body.read(MAX_PROFILE_BYTES))
            kept += 1
    return kept


def fetch(into: str | Path | None = None, *, url: str = PROFILES_ARCHIVE) -> Path:
    """Fetch the database's phone profiles, unless they are there already.

    Only the archive's ``.json`` files under :data:`PHONES` are kept, each under a name made
    from its path (never extracted by its own), each at most :data:`MAX_PROFILE_BYTES`. The
    folder is written beside, then moved onto its name once whole.

    Parameters
    ----------
    into : str or pathlib.Path, optional
        The folder (default :func:`default_folder`).
    url : str, optional
        The archive (default :data:`PROFILES_ARCHIVE`).

    Returns
    -------
    pathlib.Path
        The folder.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the archive is over :data:`MAX_ARCHIVE_BYTES`, not a gzipped tar, or holds no
        phone profile.
    OSError
        If the download fails.
    """
    folder = Path(into) if into is not None else default_folder()
    if folder.is_dir() and any(folder.glob("*.json")):
        return folder
    folder.parent.mkdir(parents=True, exist_ok=True)
    staging = folder.with_name(f".{folder.name}.{uuid.uuid4().hex}.fetching")
    staging.mkdir()
    try:
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read(MAX_ARCHIVE_BYTES + 1)
        if len(data) > MAX_ARCHIVE_BYTES:
            raise InvalidInputError(f"profiles: the archive is over {MAX_ARCHIVE_BYTES} bytes")
        try:
            with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
                kept = _keep_phones(archive, staging)
        except tarfile.TarError:
            raise InvalidInputError("profiles: not a gzipped tar") from None
        if kept == 0:
            raise InvalidInputError("profiles: the archive holds no phone profile")
        if folder.exists():
            folder.rmdir()  # an empty folder of that name: refused if not empty
        staging.replace(folder)
    except BaseException:
        shutil.rmtree(staging)  # the fetch's own, made above
        raise
    return folder


def main(arguments: Sequence[str]) -> int:
    """Fetch the profiles, or say the nominal field of view (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a one-line summary on stdout.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it_all.eval.lenses")
    commands = parser.add_subparsers(dest="command", required=True)
    fetching = commands.add_parser("fetch", help="fetch the phone profiles")
    fetching.add_argument("--into", type=Path, help="the folder (default: the user's cache)")
    measuring = commands.add_parser("nominal", help="the median main-camera field of view")
    measuring.add_argument("--profiles", type=Path, help="the profiles (default: fetched)")
    args = parser.parse_args(arguments)
    if args.command == "fetch":
        folder = fetch(args.into)
        files = len(list(folder.glob("*.json")))
        print(f"{folder}: {len(load(folder))} usable profiles of {files}")
    else:
        fov, count = nominal_fov(load(args.profiles or default_folder()))
        print(f"median diagonal field of view: {fov:.2f} degrees over {count} main-camera profiles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
