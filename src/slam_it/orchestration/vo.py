"""A whole sequence through the edge pipeline: read it, track it, write the run.

:func:`run` reads a dataset's sequence (:mod:`slam_it.eval.tum`, :mod:`slam_it.eval.euroc`),
builds each frame for the mode (:mod:`slam_it.edge.frames`), tracks it
(:class:`slam_it.edge.pipeline.Pipeline`), and writes the run's folder
(:mod:`slam_it.eval.runs`):

- **The camera.** A TUM-format folder's (its ``camera.txt``, or the sensor its name says:
  :func:`slam_it.eval.tum.read_camera`): as it is for monocular, and for RGB-D with a virtual
  baseline of the camera's ``bf`` over its focal length (ORB-SLAM's ``Camera.bf``: depth gives
  each keypoint a virtual right column that baseline away, and points nearer than
  ``stereo.close_depth`` baselines count as close); a folder whose camera gives no ``bf`` runs
  monocular only. A EuRoC sequence's left camera for monocular; for stereo, the pair rectified
  (:func:`slam_it.foundations.stereo.rectify`), each image remapped, and the rectified left
  camera (pinhole) with the pair's baseline. A stereo run's poses are therefore the rectified
  left camera's: the left camera's centre, turned by a fixed rotation. TUM has no stereo
  pairs and EuRoC no depth maps (:func:`check`).
- **The frames.** ``count`` of them from ``first`` (all by default), in the sequence's order.
  An RGB-D frame without a depth map within the reader's tolerance is skipped.
- **Each frame** gives one diagnostics line (``timestamp``, ``state``, ``map``, ``tracked``,
  ``keyframe``, the time each step took, in seconds: the run format's
  :data:`slam_it.eval.runs.STEPS`; then how its stages went:
  :meth:`slam_it.edge.pipeline.FrameSummary.as_dict`'s keys) and, if tracked, its
  ``T_world_cam`` and its map; with ``record_features``, its keypoints' pixels, levels and
  fates, streamed to the run's folder as it goes (:class:`slam_it.eval.runs.FeatureWriter`).
  A lost track closes its map and the run goes on, the next frame starting a new one
  (:class:`slam_it.edge.pipeline.Pipeline`); a map reset has its frames' poses dropped from
  the trajectory, as ORB-SLAM3 drops a reset map's. The steps, as a camera and ORB-SLAM's
  threads divide them:

  - ``read_seconds``: delivering the frame's images, decoded (a depth map in metres), and a
    stereo pair rectified, as ORB-SLAM2's examples do before they hand a frame over;
  - ``build_seconds``: building the frame from them (:mod:`slam_it.edge.frames`: the pyramid,
    the features, and the stereo matches or the depth at each keypoint);
  - ``track_seconds`` and ``map_seconds``: tracking it, and mapping it if it became a
    keyframe (:class:`slam_it.edge.pipeline.FrameResult`). Mapping here includes making the
    keyframe and, with depth, its close points, which ORB-SLAM2's tracking thread does
    (``Tracking::CreateNewKeyFrame``) before it hands the keyframe to local mapping.
- **At the end,** every keyframe and point the run made (their times, last ``T_world_cam`` and
  positions), their lives and their maps (:meth:`slam_it.edge.pipeline.Pipeline.lifetimes`).
  The run's ``sequence`` is its folder's name (the path resolved), its ``settings`` the
  pipeline's options.

Backends: ``python`` runs the reference in-process, every stage in Python, frame building
included. ``cpp`` and ``rust`` run their native program, :data:`PROGRAM`
(:func:`slam_it.backends.program`), with the request on its command line
(:meth:`Request.arguments`), then read the run back. The program's exit code names a failed
run's error kind (:data:`EXIT_FAILED`), raised here as the same exception.

**The programs.** Each backend has one (:func:`command`): the native ``slam_it_vo`` in C++ and
Rust, and this module for Python (``python -m slam_it.orchestration.vo``, :func:`main`), with
the same command line (:func:`parse`), exit codes and messages. A program records its own
process in its run before writing it: its peak memory, the CPUs it could run on and its
memory cap (:mod:`slam_it.eval.budgets`); Python's in-process run records none, since its
process holds more than the run.
"""

from __future__ import annotations

import contextlib
import re
import subprocess
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it import backends
from slam_it.edge import frames
from slam_it.edge.options import MODES, Options, features_for
from slam_it.edge.pipeline import Pipeline
from slam_it.eval import budgets, euroc, images, runs, tum, vocabularies
from slam_it.foundations import image, orb, se3, stereo
from slam_it.foundations.errors import (
    DegenerateError,
    InvalidInputError,
    NotConvergedError,
    SlamItError,
)
from slam_it.foundations.vocabulary import Vocabulary

PROGRAM = "slam_it_vo"
"""The native programs' name, in C++ (``cpp/apps/orchestration/``) and Rust
(``rust/slam-it-orchestration``), and the prefix of every program's messages."""

MODULE = "slam_it.orchestration.vo"
"""This module, which Python's program runs (``python -m``)."""

USAGE = (
    f"usage: python -m {MODULE} --dataset tum|euroc --sequence PATH --mode "
    "monocular|stereo|rgbd --out RUN_DIR [--first I] [--count N] [--vocabulary FOLDER] "
    "[--record-features] [--option KEY=VALUE ...]"
)
"""Python's program's command line: the native programs' (their ``kUsage``/``USAGE``)."""

EXIT_USAGE = 2
"""The program's exit code for a command line it cannot read."""

EXIT_FAILED = 3
"""Its exit code for a failed run, plus the error's kind: 3 for
:class:`~slam_it.foundations.errors.InvalidInputError`, 4 for
:class:`~slam_it.foundations.errors.DegenerateError`, 5 for
:class:`~slam_it.foundations.errors.NotConvergedError`."""

_ERRORS: tuple[type[SlamItError], ...] = (InvalidInputError, DegenerateError, NotConvergedError)
"""The exceptions a failed run's exit code names, from :data:`EXIT_FAILED` on."""

_REQUIRED = ("--dataset", "--sequence", "--mode", "--out")
"""The flags every command line gives."""

_RECORD_FEATURES = "--record-features"
"""The one flag without a value: record each frame's keypoints, with their fates."""


class UsageError(InvalidInputError):
    """A command line the program cannot read: :data:`EXIT_USAGE`, not a failed run."""


@dataclass(frozen=True)
class Request:
    """A run asked for: what the native program's command line says.

    Attributes
    ----------
    dataset : str
        One of :data:`slam_it.eval.runs.DATASETS`.
    sequence : pathlib.Path
        The sequence's folder.
    mode : str
        One of :data:`slam_it.edge.options.MODES`.
    out : pathlib.Path
        The run's folder, written.
    first : int
        The first frame run.
    count : int or None
        The frames run from it, or None for all.
    options : slam_it.edge.options.Options
        The pipeline's settings.
    record_features : bool
        Whether the run records each frame's keypoints, with their fates.
    vocabulary : pathlib.Path or None
        The vocabulary's folder: required with ``relocalization.enabled`` on, refused with it off.
    """

    dataset: str
    sequence: Path
    mode: str
    out: Path
    first: int = 0
    count: int | None = None
    options: Options = field(default_factory=Options)
    record_features: bool = False
    vocabulary: Path | None = None

    def arguments(self) -> list[str]:
        """Return the native program's command line for the request.

        Returns
        -------
        list of str
            The arguments after the program's name: every flag (``--vocabulary`` if given,
            ``--record-features`` if asked), and ``--option KEY=VALUE`` for each setting that
            differs from its default (flags as ``true`` or ``false``, integers as integers,
            reals round-tripping).
        """
        out = [
            "--dataset",
            self.dataset,
            "--sequence",
            str(self.sequence),
            "--mode",
            self.mode,
            "--out",
            str(self.out),
            "--first",
            str(self.first),
        ]
        if self.count is not None:
            out += ["--count", str(self.count)]
        if self.vocabulary is not None:
            out += ["--vocabulary", str(self.vocabulary)]
        if self.record_features:
            out.append("--record-features")
        defaults = Options().settings()
        for name, value in self.options.settings().items():
            default = defaults[name]
            if value != default:
                if isinstance(default, bool):
                    text = "true" if value else "false"
                elif isinstance(default, int):
                    text = str(int(value))
                else:
                    text = repr(float(value))
                out += ["--option", f"{name}={text}"]
        return out


def check(request: Request) -> None:
    """Check a request.

    Parameters
    ----------
    request : Request
        The request.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        For an unknown dataset or mode, a mode the dataset does not record (TUM has no stereo
        pairs, EuRoC no depth maps), a negative `first` or `count`, or a vocabulary missing
        while relocalising or given while not. (The options checked themselves when they were
        made; the vocabulary's folder is checked when it is read.)
    """
    if request.dataset not in runs.DATASETS:
        wanted = " or ".join(runs.DATASETS)
        raise InvalidInputError(f"dataset: expected {wanted}, got {request.dataset!r}")
    if request.mode not in MODES:
        raise InvalidInputError(f"--mode: expected monocular, stereo or rgbd, got {request.mode!r}")
    if request.dataset == "tum" and request.mode == "stereo":
        raise InvalidInputError("mode: a tum sequence has no stereo pairs")
    if request.dataset == "euroc" and request.mode == "rgbd":
        raise InvalidInputError("mode: a euroc sequence has no depth maps")
    if request.first < 0 or (request.count is not None and request.count < 0):
        raise InvalidInputError("first, count: must be at least 0")
    if request.options.relocalization.enabled and request.vocabulary is None:
        raise InvalidInputError("--vocabulary: relocalization.enabled needs one")
    if not request.options.relocalization.enabled and request.vocabulary is not None:
        raise InvalidInputError("--vocabulary: given, but relocalization.enabled is off")


def _integer(flag: str, text: str) -> int:
    """Read a whole integer as the native programs do (``eval::text::integer``)."""
    if not re.fullmatch(r"-?[0-9]+", text.strip()):
        raise UsageError(f"{flag}: expected an integer, got '{text}'")
    return int(text)


_DECIMAL = re.compile(r"[-+]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][-+]?[0-9]+)?")
"""A decimal number, as the native programs read one (``eval::text::number``)."""


def parse_setting(text: str) -> tuple[str, bool | int | float]:
    """Read one ``--option KEY=VALUE``, as the programs read it.

    Parameters
    ----------
    text : str
        ``KEY=VALUE``: a setting of :class:`~slam_it.edge.options.Options`, named by its group
        and field (``relocalization.enabled``), and its value.

    Returns
    -------
    tuple of (str, int or float)
        The setting's name and its value, of its type: a whole integer for an integer
        setting, a finite decimal number otherwise.

    Raises
    ------
    UsageError
        For no ``=``, an unknown setting, or a value that is not of its type (``true`` or
        ``false`` for a flag).
    """
    key, equals, value = text.partition("=")
    if not equals:
        raise UsageError(f"--option: expected KEY=VALUE, got '{text}'")
    defaults = Options().settings()
    if key not in defaults:
        raise UsageError(f"--option: no setting '{key}'")
    default = defaults[key]
    if isinstance(default, bool):
        if value.strip() not in ("true", "false"):
            raise UsageError(f"{key}: expected true or false, got '{value}'")
        return key, value.strip() == "true"
    if isinstance(default, int):
        return key, _integer(key, value)
    real = float(value) if _DECIMAL.fullmatch(value.strip()) else float("nan")
    if not np.isfinite(real):  # past float64's range, the native readers refuse it too
        raise UsageError(f"{key}: expected a finite number, got '{value}'")
    return key, real


def parse(arguments: Sequence[str]) -> Request:
    """Read a program's command line (:data:`USAGE`), as the native programs read theirs.

    Flags come in any order, each with its value but ``--record-features``; ``--option
    KEY=VALUE`` is repeatable and names a setting of :class:`~slam_it.edge.options.Options` by its
    group and field.

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    Request
        The request. Its settings checked themselves when they were made: a value out of
        range raises :class:`~slam_it.foundations.errors.InvalidInputError`, which a program
        reports as a failed run, as the natives' check does.

    Raises
    ------
    UsageError
        For an unknown, repeated or missing flag, a flag without its value, an unknown mode or
        setting, or a value that is not a number.
    """
    values: dict[str, str] = {}
    numbers: dict[str, int] = {}
    changes: dict[str, Any] = {}  # each of its setting's type (parse_setting)
    i = 0
    while i < len(arguments):
        flag = arguments[i]
        if flag == _RECORD_FEATURES:
            if flag in values:
                raise UsageError(f"{flag}: given twice")
            values[flag] = ""
            i += 1
            continue
        if i + 1 == len(arguments):
            raise UsageError(f"{flag}: expected a value")
        value = arguments[i + 1]
        i += 2
        if flag == "--option":
            key, setting = parse_setting(value)
            changes[key] = setting
            continue
        if flag in values:
            raise UsageError(f"{flag}: given twice")
        if flag not in (*_REQUIRED, "--first", "--count", "--vocabulary"):
            raise UsageError(f"unknown argument '{flag}'")
        if flag == "--mode" and value not in MODES:
            raise UsageError(f"--mode: expected monocular, stereo or rgbd, got '{value}'")
        if flag in ("--first", "--count"):
            numbers[flag] = _integer(flag, value)
        values[flag] = value
    for required in _REQUIRED:
        if required not in values:
            raise UsageError(f"{required}: missing")
    return Request(
        values["--dataset"],
        Path(values["--sequence"]),
        values["--mode"],
        Path(values["--out"]),
        first=numbers.get("--first", 0),
        count=numbers.get("--count"),
        options=Options().with_settings(changes),
        record_features=_RECORD_FEATURES in values,
        vocabulary=Path(values["--vocabulary"]) if "--vocabulary" in values else None,
    )


def command(backend: str) -> list[str]:
    """Return the command that runs a backend's program, before its arguments.

    Parameters
    ----------
    backend : {"python", "cpp", "rust"}
        The backend.

    Returns
    -------
    list of str
        Python's: this module, run by the interpreter running this one. The others': their
        native program (:func:`slam_it.backends.program`).

    Raises
    ------
    ValueError
        If `backend` is unknown.
    slam_it.backends.BackendNotBuiltError
        If a native program is not built.
    """
    backends.check_name(backend)
    if backend == "python":
        return [sys.executable, "-m", MODULE]
    return [str(backends.program(backend, PROGRAM))]


def options_for(width: int, height: int) -> Options:
    """Return ORB-SLAM's settings with the features for a frame's size, as a run is set up.

    :func:`slam_it.edge.options.features_for` scales the feature count with the frame's area;
    the demo's worker and the test set's measure both set a run up with this, so they apply one
    rule. ``run``'s own default stays ORB-SLAM's, so a dataset's results do not move.

    Parameters
    ----------
    width, height : int
        The frames' size, in pixels.

    Returns
    -------
    slam_it.edge.options.Options
        The settings.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a side is not a positive integer.
    """
    return Options().with_settings({"features.n_features": features_for(width, height)})


def run(  # noqa: PLR0913 -- the sequence, its mode, the run's folder; the rest keyword-only
    dataset: str,
    sequence: str | Path,
    mode: str,
    out: str | Path,
    *,
    backend: str = "python",
    options: Options | None = None,
    first: int = 0,
    count: int | None = None,
    budget: budgets.Budget | None = None,
    record_features: bool = False,
    vocabulary: str | Path | None = None,
) -> runs.Run:
    """Run a sequence through the edge pipeline and write its run folder.

    Parameters
    ----------
    dataset : {"tum", "euroc"}
        The dataset the sequence comes from.
    sequence : str or pathlib.Path
        The sequence's folder: a TUM-format folder, or a EuRoC sequence (holding ``mav0``).
    mode : {"monocular", "stereo", "rgbd"}
        The sensor mode.
    out : str or pathlib.Path
        The run's folder (created if needed; its files are replaced).
    backend : {"python", "cpp", "rust"}, optional
        Which pipeline runs: Python's in-process, or a native program.
    options : slam_it.edge.options.Options, optional
        The pipeline's settings (default: ORB-SLAM's).
    first : int, optional
        The first frame run.
    count : int, optional
        The frames run from it (default: all).
    budget : slam_it.eval.budgets.Budget, optional
        A compute budget to run within: the backend's program (Python's too, rather than the
        in-process run) pinned to the budget's cores and its memory capped
        (:func:`slam_it.eval.budgets.command`).
    record_features : bool, optional
        Whether to record each frame's keypoints, with their fates
        (:func:`slam_it.eval.runs.read_features` reads them; default: no).
    vocabulary : str or pathlib.Path, optional
        The vocabulary's folder, which a relocalising run needs (default: the committed one when
        ``relocalization.enabled`` is on, none when it is off; one given while it is off is
        refused). It is read once, here, its checksums checked before any program runs
        (:func:`slam_it.eval.vocabularies.read`).

    Returns
    -------
    slam_it.eval.runs.Run
        The run, as read back from its folder.

    Raises
    ------
    ValueError
        If `backend` is unknown.
    slam_it.backends.BackendNotBuiltError
        If the backend's program is not built (:class:`~slam_it.backends.StaleProgramError`:
        a Rust program older than a file it is built from).
    slam_it.foundations.errors.SlamItError
        The request's check (:func:`check`), or a reader's, the pipeline's or the writer's
        error, of the same kind whichever backend ran.
    FileNotFoundError
        From the Python backend in-process, if a file of the sequence is missing (the
        programs raise :class:`~slam_it.foundations.errors.InvalidInputError`, their only
        kind for it).
    RuntimeError
        If a program dies otherwise: killed, for one, past a budget's memory.
    """
    request = Request(
        dataset,
        Path(sequence),
        mode,
        Path(out),
        first=first,
        count=count,
        options=Options() if options is None else options,
        record_features=record_features,
        vocabulary=None if vocabulary is None else Path(vocabulary),
    )
    if request.vocabulary is None and request.options.relocalization.enabled:
        request = replace(request, vocabulary=vocabularies.DEFAULT)
    backends.check_name(backend)
    check(request)
    words = _read_vocabulary(request)  # its versions and checksums, before any program runs
    if backend == "python" and budget is None:
        runs.write_run(request.out, _run_here(request, words))
    else:
        _run_program(backend, request, budget)
    return runs.read_run(request.out)


@dataclass(frozen=True)
class Delivered:
    """A frame's images as a camera delivers them: decoded, and a stereo pair rectified.

    A monocular frame has its image only, a stereo frame the right image too, and an RGB-D
    frame its depth map, in metres.

    Attributes
    ----------
    timestamp : float
        The frame's time, in seconds.
    image : ndarray of uint8, shape (H, W)
        Its gray image (the left one, rectified, for stereo).
    right : ndarray of uint8, shape (H, W), or None
        A stereo frame's right image, rectified.
    depth : ndarray of float64, shape (H, W), or None
        An RGB-D frame's depth map, in metres.
    """

    timestamp: float
    image: NDArray[np.uint8]
    right: NDArray[np.uint8] | None = None
    depth: NDArray[np.float64] | None = None


@dataclass(frozen=True)
class Source:
    """A sequence opened for a run (:func:`open_source`): its camera and frames.

    Attributes
    ----------
    camera : slam_it.edge.frames.Camera
        The camera the pipeline runs with: a stereo pair's rectified left camera.
    times : ndarray of float64, shape (F,)
        Each frame's time, in seconds, in the sequence's order.
    read : callable
        ``read(i)``: frame ``i``'s images as delivered (:class:`Delivered`); None for a frame
        the run skips (an RGB-D frame without a depth map).
    view : callable
        ``view(i)``: frame ``i``'s image as the pipeline sees it (a stereo pair's left image
        rectified), in colour where the sequence has colour: uint8, (H, W) or (H, W, 3).
    mask : ndarray of uint8, shape (H, W), or None
        The features every frame drops, those on a 0 (a TUM-format folder's
        ``mask.png``, :func:`slam_it.eval.tum.read_mask`); None keeps them all.
    """

    camera: frames.Camera
    times: NDArray[np.float64]
    read: Callable[[int], Delivered | None]
    view: Callable[[int], NDArray[np.uint8]]
    mask: NDArray[np.uint8] | None = None


def _build(
    delivered: Delivered,
    camera: frames.Camera,
    extraction: orb.Options,
    matching: stereo.Options,
    mask: NDArray[np.uint8] | None,
) -> frames.Frame:
    """Build a frame from its images, for the mode they were delivered for, masked if asked."""
    if delivered.right is not None:
        return frames.stereo_pair(
            delivered.image, delivered.right, delivered.timestamp, camera, extraction,
            stereo_options=matching, mask=mask,
        )  # fmt: skip
    if delivered.depth is not None:
        return frames.rgbd(
            delivered.image, delivered.depth, delivered.timestamp, camera, extraction, mask=mask
        )
    return frames.monocular(delivered.image, delivered.timestamp, camera, extraction, mask=mask)


def _tum_source(root: Path, mode: str) -> Source:
    """Open a TUM-format folder: its camera, an RGB-D one with the virtual baseline."""
    read = tum.read_frames(root)
    sensor = read.camera
    baseline = 0.0
    if mode == "rgbd":
        if sensor.bf is None:
            raise InvalidInputError(
                f"{root}: an RGB-D run needs the depth sensor's bf: add a 'bf' line to "
                f"{tum.CAMERA_FILE}"
            )
        baseline = sensor.bf / float(sensor.intrinsics[0])
    camera = frames.Camera(sensor.model, sensor.intrinsics, sensor.width, sensor.height, baseline)

    def deliver(i: int) -> Delivered | None:
        gray = images.read_gray(read.images[i])
        timestamp = float(read.timestamps[i])
        if mode == "monocular":
            return Delivered(timestamp, gray)
        depth_file = read.depths[i]
        if depth_file is None:
            return None
        return Delivered(timestamp, gray, depth=tum.read_depth(depth_file, sensor.depth_scale))

    def view(i: int) -> NDArray[np.uint8]:
        return images.read_rgb(read.images[i])

    return Source(camera, np.asarray(read.timestamps, dtype=np.float64), deliver, view, read.mask)


def _stereo_source(read: euroc.Frames) -> Source:
    """Open a EuRoC sequence's stereo pairs, rectified: the rectified left camera."""
    left, right = read.left_camera, read.right_camera
    rectification = stereo.rectify(
        "radtan", left.intrinsics, "radtan", right.intrinsics, read.T_right_left, shape=left.shape
    )
    maps = stereo.rectification_maps(rectification)
    rows, cols = left.shape
    camera = frames.Camera("pinhole", rectification.intrinsics, cols, rows, rectification.baseline)

    def deliver(i: int) -> Delivered:
        left_image, right_image = (
            image.remap(images.read_gray(path), coordinates)
            for path, coordinates in zip((read.left[i], read.right[i]), maps, strict=True)
        )
        return Delivered(float(read.timestamps[i]), left_image, right=right_image)

    def view(i: int) -> NDArray[np.uint8]:
        return image.remap(images.read_gray(read.left[i]), maps[0])

    return Source(camera, np.asarray(read.timestamps, dtype=np.float64), deliver, view)


def _euroc_source(root: Path, mode: str) -> Source:
    """Open a EuRoC sequence: its left camera for monocular, the pairs rectified for stereo."""
    read = euroc.read_frames(root)
    if mode == "stereo":
        return _stereo_source(read)
    left = read.left_camera
    rows, cols = left.shape
    camera = frames.Camera("radtan", left.intrinsics, cols, rows)

    def deliver(i: int) -> Delivered:
        return Delivered(float(read.timestamps[i]), images.read_gray(read.left[i]))

    def view(i: int) -> NDArray[np.uint8]:
        return images.read_gray(read.left[i])

    return Source(camera, np.asarray(read.timestamps, dtype=np.float64), deliver, view)


def open_source(dataset: str, sequence: str | Path, mode: str) -> Source:
    """Open a dataset's sequence as a run reads it.

    Parameters
    ----------
    dataset : str
        The dataset, one of :data:`slam_it.eval.runs.DATASETS`.
    sequence : str or pathlib.Path
        The sequence's folder.
    mode : {"monocular", "stereo", "rgbd"}
        The sensor mode (checked with the dataset by :func:`check`).

    Returns
    -------
    Source
        The camera, the frames' times, and their readers.

    Raises
    ------
    FileNotFoundError
        If a file of the sequence is missing.
    slam_it.foundations.errors.InvalidInputError
        If the dataset is not one of :data:`slam_it.eval.runs.DATASETS`, a file is
        malformed, or an RGB-D sequence's camera gives no ``bf``.
    """
    root = Path(sequence)
    if dataset == "tum":
        return _tum_source(root, mode)
    if dataset == "euroc":
        return _euroc_source(root, mode)
    raise InvalidInputError(f"dataset: expected one of {runs.DATASETS}, got {dataset!r}")


def _read_vocabulary(request: Request) -> Vocabulary | None:
    """Read the request's vocabulary, if it names one (:func:`slam_it.eval.vocabularies.read`)."""
    return None if request.vocabulary is None else vocabularies.read(request.vocabulary)


def _run_here(request: Request, words: Vocabulary | None) -> runs.Run:
    """Run the request with the Python pipeline, in-process, with its vocabulary as read."""
    source = open_source(request.dataset, request.sequence, request.mode)
    matching = request.options.stereo_matching()
    tracker = Pipeline(request.mode, source.camera, request.options, words)
    end = len(source.times)
    if request.count is not None:
        end = min(end, request.first + request.count)
    times: list[float] = []
    poses: list[NDArray[np.float64]] = []
    pose_maps: list[int] = []
    diagnostics: list[dict[str, object]] = []
    recording: contextlib.AbstractContextManager[runs.FeatureWriter | None] = (
        runs.FeatureWriter(request.out) if request.record_features else contextlib.nullcontext()
    )
    with recording as features:
        for i in range(request.first, end):
            start = time.perf_counter()
            delivered = source.read(i)
            after_read = time.perf_counter()
            if delivered is None:
                continue
            frame = _build(delivered, source.camera, tracker.extraction(), matching, source.mask)
            after_build = time.perf_counter()
            result = tracker.track(frame)
            diagnostics.append(
                {
                    "timestamp": result.timestamp,
                    "state": result.state.value,
                    "map": result.map,
                    "tracked": result.tracked,
                    "keyframe": result.keyframe,
                    "read_seconds": after_read - start,
                    "build_seconds": after_build - after_read,
                    "track_seconds": result.track_seconds,
                    "map_seconds": result.map_seconds,
                    **result.summary.as_dict(),
                }
            )
            if features is not None:
                keypoints = frame.features.keypoints
                features.write(keypoints.xy, keypoints.level, result.fates.fate, result.fates.point)
            if result.pose is not None and result.map is not None:
                poses.append(se3.inverse(result.pose))
                times.append(result.timestamp)
                pose_maps.append(result.map)
            lost = result.summary.lost
            if lost is not None and lost.reset:  # its frames' poses go with it
                while pose_maps and pose_maps[-1] == result.map:
                    del poses[-1], times[-1], pose_maps[-1]
    lives = tracker.lifetimes()
    keyframes = lives.keyframe_poses
    return runs.Run(
        dataset=request.dataset,
        sequence=request.sequence.resolve().name,
        mode=request.mode,
        backend="python",
        timestamps=np.array(times),
        poses=np.array(poses).reshape(-1, 4, 4),
        keyframe_timestamps=lives.keyframe_times,
        keyframe_poses=se3.inverse(keyframes) if len(keyframes) else np.zeros((0, 4, 4)),
        points=lives.points,
        settings=request.options.as_dict(),
        vocabulary=None if request.vocabulary is None else vocabularies.record(request.vocabulary),
        diagnostics=diagnostics,
        point_lives=lives.point_lives,
        keyframe_lives=lives.keyframe_lives,
        keyframe_parents=lives.keyframe_parents,
        keyframe_links=lives.keyframe_links,
        record_features=request.record_features,
        pose_maps=np.array(pose_maps, dtype=np.int64),
        keyframe_maps=lives.keyframe_maps,
        point_maps=lives.point_maps,
    )


def _run_program(backend: str, request: Request, budget: budgets.Budget | None) -> None:
    """Run the request with a backend's program (:func:`command`), within a budget if given."""
    program = command(backend)
    argv = [*program, *request.arguments()]
    if budget is not None:
        argv = budgets.command(budget, budgets.cpus(budget), argv)
    done = subprocess.run(argv, capture_output=True, text=True, check=False)
    if done.returncode == 0:
        return
    if done.returncode < 0:
        within = "" if budget is None else f", within {budget.name}'s {budget.memory} bytes"
        raise RuntimeError(f"{' '.join(program)} was killed by signal {-done.returncode}{within}")
    lines = done.stderr.strip().splitlines()
    message = lines[0].removeprefix(f"{PROGRAM}: ") if lines else ""
    kind = done.returncode - EXIT_FAILED
    if done.returncode == EXIT_USAGE:
        raise InvalidInputError(message)
    if 0 <= kind < len(_ERRORS):
        raise _ERRORS[kind](message)
    raise RuntimeError(
        f"{' '.join(program)} failed (exit code {done.returncode}): {done.stderr.strip()}"
    )


def main(arguments: Sequence[str]) -> int:
    """Run Python's program: the native programs' command line, exit codes and messages.

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name (:data:`USAGE`).

    Returns
    -------
    int
        0 for a run written, with a one-line summary on stdout; :data:`EXIT_USAGE` for a
        command line it cannot read, with the usage; :data:`EXIT_FAILED` plus the error's kind
        for a failed run (a file it cannot read or write is ``InvalidInput``, the native
        programs' kind for it). Messages go to stderr, each prefixed with :data:`PROGRAM`.
    """
    try:
        request = parse(arguments)
        check(request)
        # What the program records of its own process, before it writes the run.
        done = replace(
            _run_here(request, _read_vocabulary(request)),
            peak_memory=budgets.peak_memory(),
            cpus=budgets.allowed_cpus(),
            memory_cap=budgets.memory_cap(),
        )
        runs.write_run(request.out, done)
    except UsageError as error:
        print(f"{PROGRAM}: {error}\n{USAGE}", file=sys.stderr)
        return EXIT_USAGE
    except OSError as error:
        print(f"{PROGRAM}: {error}", file=sys.stderr)
        return EXIT_FAILED
    except SlamItError as error:
        print(f"{PROGRAM}: {error}", file=sys.stderr)
        return EXIT_FAILED + next(k for k, kind in enumerate(_ERRORS) if isinstance(error, kind))
    print(
        f"{PROGRAM}: {len(done.timestamps)} of {len(done.diagnostics)} frames tracked, "
        f"{len(done.final_keyframes()[0])} keyframes, {len(done.final_points())} points "
        f"-> {request.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
