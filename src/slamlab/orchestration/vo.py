"""A whole sequence through the edge pipeline: read it, track it, write the run.

:func:`run` reads a dataset's sequence (:mod:`slamlab.eval.tum`, :mod:`slamlab.eval.euroc`),
builds each frame for the mode (:mod:`slamlab.edge.frames`), tracks it
(:class:`slamlab.edge.pipeline.Pipeline`), and writes the run's folder
(:mod:`slamlab.eval.runs`):

- **The camera.** A TUM-format folder's (its ``camera.txt``, or the sensor its name says:
  :func:`slamlab.eval.tum.read_camera`): as it is for monocular, and for RGB-D with a virtual
  baseline of the camera's ``bf`` over its focal length (ORB-SLAM's ``Camera.bf``: depth gives
  each keypoint a virtual right column that baseline away, and points nearer than
  ``close_depth`` baselines count as close); a folder whose camera gives no ``bf`` runs
  monocular only. A EuRoC sequence's left camera for monocular; for stereo, the pair rectified
  (:func:`slamlab.foundations.stereo.rectify`), each image remapped, and the rectified left
  camera (pinhole) with the pair's baseline. A stereo run's poses are therefore the rectified
  left camera's: the left camera's centre, turned by a fixed rotation. TUM has no stereo
  pairs and EuRoC no depth maps (:func:`check`).
- **The frames.** ``count`` of them from ``first`` (all by default), in the sequence's order.
  An RGB-D frame without a depth map within the reader's tolerance is skipped.
- **Each frame** gives one diagnostics line (``timestamp``, ``state``, ``tracked``,
  ``keyframe``, ``seconds``) and, if tracked, its ``T_world_cam``. The run ends at the frame
  that loses the track.
- **At the end,** the map's keyframes (their times and ``T_world_cam``) and points. The run's
  ``sequence`` is its folder's name (the path resolved), its ``settings`` the pipeline's
  options.

Backends: ``python`` runs the reference in-process, every stage in Python, frame building
included. ``cpp`` and ``rust`` run their native program, :data:`PROGRAM`
(:func:`slamlab.backends.program`), with the request on its command line
(:meth:`Request.arguments`), then read the run back. The program's exit code names a failed
run's error kind (:data:`EXIT_FAILED`), raised here as the same exception.
"""

from __future__ import annotations

import subprocess
from collections.abc import Callable
from dataclasses import dataclass, field, fields
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.edge import frames
from slamlab.edge.options import MODES, Options
from slamlab.edge.pipeline import Pipeline, State
from slamlab.eval import euroc, images, runs, tum
from slamlab.foundations import image, orb, se3, stereo
from slamlab.foundations.errors import (
    DegenerateError,
    InvalidInputError,
    NotConvergedError,
    SlamlabError,
)

DATASETS = ("tum", "euroc")
"""The datasets a sequence can come from."""

PROGRAM = "slamlab_vo"
"""The native programs' name, in C++ (``cpp/apps/orchestration/``) and Rust
(``rust/slamlab-orchestration``)."""

EXIT_USAGE = 2
"""The program's exit code for a command line it cannot read."""

EXIT_FAILED = 3
"""Its exit code for a failed run, plus the error's kind: 3 for
:class:`~slamlab.foundations.errors.InvalidInputError`, 4 for
:class:`~slamlab.foundations.errors.DegenerateError`, 5 for
:class:`~slamlab.foundations.errors.NotConvergedError`."""

_ERRORS: tuple[type[SlamlabError], ...] = (InvalidInputError, DegenerateError, NotConvergedError)
"""The exceptions a failed run's exit code names, from :data:`EXIT_FAILED` on."""


@dataclass(frozen=True)
class Request:
    """A run asked for: what the native program's command line says.

    Attributes
    ----------
    dataset : str
        One of :data:`DATASETS`.
    sequence : pathlib.Path
        The sequence's folder.
    mode : str
        One of :data:`slamlab.edge.options.MODES`.
    out : pathlib.Path
        The run's folder, written.
    first : int
        The first frame run.
    count : int or None
        The frames run from it, or None for all.
    options : slamlab.edge.options.Options
        The pipeline's settings.
    """

    dataset: str
    sequence: Path
    mode: str
    out: Path
    first: int = 0
    count: int | None = None
    options: Options = field(default_factory=Options)

    def arguments(self) -> list[str]:
        """Return the native program's command line for the request.

        Returns
        -------
        list of str
            The arguments after the program's name: every flag, and ``--option KEY=VALUE``
            for each setting that differs from its default (integers as integers, reals
            round-tripping).
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
        defaults = Options()
        for setting in fields(Options):
            value, default = getattr(self.options, setting.name), getattr(defaults, setting.name)
            if value != default:
                text = str(int(value)) if isinstance(default, int) else repr(float(value))
                out += ["--option", f"{setting.name}={text}"]
        return out


def check(request: Request) -> None:
    """Check a request.

    Parameters
    ----------
    request : Request
        The request.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        For an unknown dataset or mode, a mode the dataset does not record (TUM has no stereo
        pairs, EuRoC no depth maps), or a negative `first` or `count`. (The options checked
        themselves when they were made.)
    """
    if request.dataset not in DATASETS:
        raise InvalidInputError(f"dataset: expected tum or euroc, got {request.dataset!r}")
    if request.mode not in MODES:
        raise InvalidInputError(f"--mode: expected monocular, stereo or rgbd, got {request.mode!r}")
    if request.dataset == "tum" and request.mode == "stereo":
        raise InvalidInputError("mode: a tum sequence has no stereo pairs")
    if request.dataset == "euroc" and request.mode == "rgbd":
        raise InvalidInputError("mode: a euroc sequence has no depth maps")
    if request.first < 0 or (request.count is not None and request.count < 0):
        raise InvalidInputError("first, count: must be at least 0")


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
    options : slamlab.edge.options.Options, optional
        The pipeline's settings (default: ORB-SLAM's).
    first : int, optional
        The first frame run.
    count : int, optional
        The frames run from it (default: all).

    Returns
    -------
    slamlab.eval.runs.Run
        The run, as read back from its folder.

    Raises
    ------
    ValueError
        If `backend` is unknown.
    slamlab.backends.BackendNotBuiltError
        If the backend's program is not built.
    slamlab.foundations.errors.SlamlabError
        The request's check (:func:`check`), or a reader's, the pipeline's or the writer's
        error, of the same kind whichever backend ran.
    FileNotFoundError
        From the Python backend, if a file of the sequence is missing (the native programs
        raise :class:`~slamlab.foundations.errors.InvalidInputError`, their only kind for it).
    """
    request = Request(
        dataset,
        Path(sequence),
        mode,
        Path(out),
        first=first,
        count=count,
        options=Options() if options is None else options,
    )
    backends.check_name(backend)
    check(request)
    if backend == "python":
        runs.write_run(request.out, _run_here(request))
    else:
        _run_program(backend, request)
    return runs.read_run(request.out)


_Builder = Callable[[int, orb.Options], frames.Frame | None]
"""Builds the frame at an index with the extraction settings the pipeline asks for: None for a
frame the run skips."""


@dataclass(frozen=True)
class _Source:
    """A sequence opened for a run: the camera, its frames, and how to build each."""

    camera: frames.Camera
    count: int
    frame: _Builder


def _tum_source(root: Path, mode: str) -> _Source:
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

    def build(i: int, extraction: orb.Options) -> frames.Frame | None:
        gray = images.read_gray(read.images[i])
        timestamp = float(read.timestamps[i])
        if mode == "monocular":
            return frames.monocular(gray, timestamp, camera, extraction)
        depth_file = read.depths[i]
        if depth_file is None:
            return None
        depth = tum.read_depth(depth_file, sensor.depth_scale)
        return frames.rgbd(gray, depth, timestamp, camera, extraction)

    return _Source(camera, len(read.images), build)


def _stereo_source(read: euroc.Frames, matching: stereo.Options) -> _Source:
    """Open a EuRoC sequence's stereo pairs, rectified: the rectified left camera."""
    left, right = read.left_camera, read.right_camera
    rectification = stereo.rectify(
        "radtan", left.intrinsics, "radtan", right.intrinsics, read.T_right_left, shape=left.shape
    )
    maps = stereo.rectification_maps(rectification)
    rows, cols = left.shape
    camera = frames.Camera("pinhole", rectification.intrinsics, cols, rows, rectification.baseline)

    def build(i: int, extraction: orb.Options) -> frames.Frame:
        left_image, right_image = (
            image.remap(images.read_gray(path), coordinates)
            for path, coordinates in zip((read.left[i], read.right[i]), maps, strict=True)
        )
        timestamp = float(read.timestamps[i])
        return frames.stereo_pair(
            left_image, right_image, timestamp, camera, extraction, stereo_options=matching
        )

    return _Source(camera, len(read.left), build)


def _euroc_source(root: Path, mode: str, matching: stereo.Options) -> _Source:
    """Open a EuRoC sequence: its left camera for monocular, the pairs rectified for stereo."""
    read = euroc.read_frames(root)
    if mode == "stereo":
        return _stereo_source(read, matching)
    left = read.left_camera
    rows, cols = left.shape
    camera = frames.Camera("radtan", left.intrinsics, cols, rows)

    def build(i: int, extraction: orb.Options) -> frames.Frame:
        gray = images.read_gray(read.left[i])
        return frames.monocular(gray, float(read.timestamps[i]), camera, extraction)

    return _Source(camera, len(read.left), build)


def _run_here(request: Request) -> runs.Run:
    """Run the request with the Python pipeline, in-process."""
    if request.dataset == "tum":
        source = _tum_source(request.sequence, request.mode)
    else:
        source = _euroc_source(request.sequence, request.mode, request.options.stereo_matching())
    tracker = Pipeline(request.mode, source.camera, request.options)
    end = source.count
    if request.count is not None:
        end = min(end, request.first + request.count)
    times: list[float] = []
    poses: list[NDArray[np.float64]] = []
    diagnostics: list[dict[str, object]] = []
    for i in range(request.first, end):
        frame = source.frame(i, tracker.extraction())
        if frame is None:
            continue
        result = tracker.track(frame)
        diagnostics.append(
            {
                "timestamp": result.timestamp,
                "state": result.state.value,
                "tracked": result.tracked,
                "keyframe": result.keyframe,
                "seconds": result.seconds,
            }
        )
        if result.pose is not None:
            poses.append(se3.inverse(result.pose))
            times.append(result.timestamp)
        if result.state is State.LOST:
            break
    m = tracker.map
    return runs.Run(
        dataset=request.dataset,
        sequence=request.sequence.resolve().name,
        mode=request.mode,
        backend="python",
        timestamps=np.array(times),
        poses=np.array(poses).reshape(-1, 4, 4),
        keyframe_timestamps=np.zeros(0) if m is None else m.kf_time,
        keyframe_poses=np.zeros((0, 4, 4)) if m is None else se3.inverse(m.kf_pose),
        points=np.zeros((0, 3)) if m is None else m.pt_xyz,
        settings=request.options.as_dict(),
        diagnostics=diagnostics,
    )


def _run_program(backend: str, request: Request) -> None:
    """Run the request with a compiled backend's native program."""
    program = backends.program(backend, PROGRAM)
    done = subprocess.run(
        [str(program), *request.arguments()], capture_output=True, text=True, check=False
    )
    if done.returncode == 0:
        return
    lines = done.stderr.strip().splitlines()
    message = lines[0].removeprefix(f"{PROGRAM}: ") if lines else ""
    kind = done.returncode - EXIT_FAILED
    if done.returncode == EXIT_USAGE:
        raise InvalidInputError(message)
    if 0 <= kind < len(_ERRORS):
        raise _ERRORS[kind](message)
    raise RuntimeError(f"{program} failed (exit code {done.returncode}): {done.stderr.strip()}")
