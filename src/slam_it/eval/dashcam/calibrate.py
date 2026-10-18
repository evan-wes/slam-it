"""Calibrating a front and rear dashcam as a rig, from each camera's own motion.

The two cameras face opposite ways and never see the same thing at once, so they are calibrated
as a hand-eye pair (Tsai and Lenz 1989; Park and Martin 1994): from each one's motion over the
same seconds, their clocks first lined up by their rotation rates, as Taylor and Nieto's
motion-based calibration lines them up (T-RO 2016):

1. **Each camera alone** (:func:`reconstruct`, COLMAP through ``pycolmap``, the optional
   ``dashcam`` extra): frames a few a second (:func:`extract_frames`), keypoints on the static
   mask dropped, SIFT matched to each frame's next few only (sequential: a garage's levels look
   alike, and matching by appearance across them folds the spiral onto itself), then
   incremental SfM with the lens self-calibrated, ``OPENCV_FISHEYE`` (Kannala-Brandt).
2. **The clocks** (:func:`time_offset`): a rigid rig turns as one, so the two cameras' turn over
   :data:`WINDOW` correlates best where the rear's times are shifted onto the front's; searched
   on a grid, then to :data:`OFFSET_STEP`, the rear's turn interpolated between its frames.
3. **The rotation** (:func:`hand_eye_rotation`): each pair of relative motions ``A`` (front) and
   ``B`` (rear) over a window obeys ``A X = X B`` for the rig's ``X = T_front_rear``, so the
   rotation axes of those turning at least :data:`MIN_TURN` obey ``a = R_X b``, as do the
   directions of travel of those that move straight; Kabsch's least squares fits ``R_X`` to
   both, each correspondence's residual kept.
4. **The translation and the rear map's scale** (:func:`rig_translation`): with ``R_X`` known,
   ``(R_A - I) t - s R_X t_B = -f t_A`` is linear in the rig's translation ``t`` (metres) and the
   rear map's metres per unit ``s``, the front map's ``f`` set by the GPS track's speed
   (:func:`gps_scale`). A drive on level ground leaves ``t``'s vertical part weakly seen, so a
   Gaussian prior on ``t`` (the measured baseline) joins it; the problem is linear and Gaussian,
   so its maximum a posteriori estimate and covariance are closed form. A rig bundle adjustment
   (GTSAM, once B3 brings it) is what would refine all of it together.

Every pose is ``T_cam_world``, keyed by its frame's time on its camera's overlay clock
(:mod:`slam_it.eval.dashcam.overlay`).
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import gnss, images, lenses, video
from slam_it.eval.dashcam.drives import Segment, skip
from slam_it.eval.readers import COLMAP_PIXEL_OFFSET
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
Poses = Mapping[float, Array]

WINDOW = 1.0
"""The time a relative motion spans, in seconds."""

MIN_TURN = math.radians(2.0)
"""The least turn of a pair's front motion for its rotation axis to count."""

OFFSET_SEARCH = 3.0
"""How far either way the clocks' offset is searched, in seconds."""

OFFSET_COARSE = 0.05
"""The coarse search's step, in seconds."""

OFFSET_STEP = 0.01
"""The offset's resolution, in seconds."""

MIN_RATES = 10
"""The fewest rotation rates, on each side, the offset is correlated over."""

FPS = 4.0
"""Frames reconstructed a second."""

WIDTH = 1600
"""The width frames are reconstructed at, in pixels."""

FOCAL_FACTOR = 0.5
"""The focal length COLMAP starts from, as a fraction of the width (a wide dashcam lens's)."""

MIN_SPEED = 2.0
"""The least GPS speed, in m/s, of a window that scales the front map (:func:`gps_scale`)."""

ASPECT_TOLERANCE = 0.01
"""How far ``fx / fy`` may be from 1 for a square-pixel lens (``fx = fy``) to fit."""

PRIOR_SIGMA = (0.3, 0.3, 0.15)
"""The baseline prior's standard deviation, metres, front camera's x (right), y (down), z
(forward): the tape measure's distance is better known than where each lens sits."""


@dataclass(frozen=True)
class Lens:
    """A camera's self-calibrated lens, Kannala-Brandt (``OPENCV_FISHEYE``), as COLMAP gives it.

    Attributes
    ----------
    fx, fy, cx, cy : float
        Focal lengths and principal point, in pixels at ``width`` x ``height``, COLMAP's
        convention (the top-left pixel's centre at (0.5, 0.5)).
    k : tuple of float
        The distortion's k1 to k4 (they act on the angle, so do not scale with the size).
    width, height : int
        The frame size they describe.
    error : float
        The reconstruction's mean reprojection error, in pixels.
    frames, registered : int
        The frames given, and those the reconstruction placed.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    k: tuple[float, float, float, float]
    width: int
    height: int
    error: float
    frames: int
    registered: int

    def fisheye(self, width: int, height: int) -> lenses.Fisheye:
        """Return the lens as SLAM It! models it (pixel centres at integers), at a frame size.

        Parameters
        ----------
        width, height : int
            The size wanted: the video's, of the frames' aspect.

        Returns
        -------
        slam_it.eval.lenses.Fisheye
            The model.
        """
        offset = COLMAP_PIXEL_OFFSET
        here = lenses.Pinhole(
            self.fx, self.fy, self.cx - offset, self.cy - offset, self.width, self.height
        )
        return lenses.Fisheye(here, self.k).scaled(width, height)

    @property
    def aspect(self) -> float:
        """``fx / fy``: 1 for square pixels."""
        return self.fx / self.fy


def extract_frames(
    segments: Sequence[Segment], folder: Path, *, lag: float, fps: float = FPS, width: int = WIDTH
) -> dict[str, float]:
    """Write a camera's frames, `fps` a second, as PNGs named by their time on its clock (ms).

    A damaged segment is skipped whole, the frames written from it removed, with a warning
    (:func:`.drives.skip`).

    Parameters
    ----------
    segments : sequence of Segment
        One camera's segments.
    folder : pathlib.Path
        Where the PNGs go (created).
    lag : float
        The camera's overlay clock less its segments' names' (:mod:`.overlay`), in seconds.
    fps : float, optional
        Frames a second.
    width : int, optional
        Their width, the aspect kept.

    Returns
    -------
    dict of str to float
        Each file's name and its frame's time, in seconds.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If every segment is damaged.
    """
    folder.mkdir(parents=True, exist_ok=True)
    names: dict[str, float] = {}
    damaged = 0
    for segment in segments:
        written: list[str] = []
        try:
            for elapsed, frame in video.sample_frames(segment.path, every=1.0 / fps, width=width):
                clock = segment.seconds + lag + elapsed
                name = f"{round(clock * 1000):d}.png"
                if name not in names:
                    images.write_png(folder / name, frame)
                    names[name] = clock
                    written.append(name)
        except video.DamagedSegmentError as error:
            for name in written:  # this segment's own frames, written just now
                (folder / name).unlink()
                del names[name]
            skip(segment.path, error)
            damaged += 1
    if damaged == len(segments):
        raise InvalidInputError("segments: every one is damaged")
    return names


def reconstruct(  # noqa: PLR0913 -- the frames, the work folder; how, keyword-only
    folder: Path,
    work: Path,
    *,
    mask: NDArray[np.uint8] | None = None,
    overlap: int = 10,
    seed: int = 0,
    threads: int = 4,
) -> tuple[dict[str, Array], Lens]:
    """Reconstruct one camera's frames with COLMAP, its lens self-calibrated (the module docstring).

    Parameters
    ----------
    folder : pathlib.Path
        The frames (:func:`extract_frames`), all one size.
    work : pathlib.Path
        A new or empty folder for COLMAP's database, the mask and the models.
    mask : ndarray of uint8, shape (H, W), optional
        The static mask at the frames' size, 0 where keypoints are dropped.
    overlap : int, optional
        Each frame is matched to its next `overlap`.
    seed : int, optional
        The mapper's random seed.
    threads : int, optional
        Threads COLMAP uses.

    Returns
    -------
    tuple
        Each placed frame's ``T_cam_world`` (4, 4) by file name, from the model that placed
        most, and the lens.

    Raises
    ------
    ImportError
        If ``pycolmap`` (the ``dashcam`` extra) is missing.
    slam_it.foundations.errors.InvalidInputError
        If there are fewer than 3 frames, the work folder is not empty, the mask is not the
        frames' size, or COLMAP places nothing.
    """
    import pycolmap  # noqa: PLC0415 -- the optional extra, imported where it is used

    names = sorted(p.name for p in folder.glob("*.png"))
    if len(names) < 3:
        raise InvalidInputError(f"{folder}: fewer than 3 frames")
    work.mkdir(parents=True, exist_ok=True)
    if any(work.iterdir()):
        raise InvalidInputError(f"{work}: expected a new or empty folder")
    width, height = images.size(folder / names[0])
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "OPENCV_FISHEYE"
    focal = FOCAL_FACTOR * width
    reader.camera_params = f"{focal},{focal},{width / 2},{height / 2},0,0,0,0"
    if mask is not None:
        if mask.shape != (height, width):
            raise InvalidInputError(f"mask: expected {width}x{height}, got {mask.shape[::-1]}")
        images.write_png(work / "mask.png", mask)  # COLMAP's too: 0 drops a keypoint
        reader.camera_mask_path = str(work / "mask.png")
    extraction = pycolmap.FeatureExtractionOptions()
    extraction.num_threads = threads
    database = work / "database.db"
    pycolmap.extract_features(
        database,
        folder,
        image_names=names,
        camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=reader,
        extraction_options=extraction,
        device=pycolmap.Device.cpu,
    )
    pairing = pycolmap.SequentialPairingOptions()
    pairing.overlap = overlap
    pycolmap.match_sequential(database, pairing_options=pairing, device=pycolmap.Device.cpu)
    options = pycolmap.IncrementalPipelineOptions()
    options.ba_refine_focal_length = True
    options.ba_refine_extra_params = True
    options.random_seed = seed
    options.num_threads = threads
    models = work / "models"
    models.mkdir()
    found = pycolmap.incremental_mapping(database, folder, models, options)
    if not found:
        raise InvalidInputError(f"{folder}: COLMAP placed no frames")
    model = max(found.values(), key=lambda m: m.num_reg_images())
    poses: dict[str, Array] = {}
    for image in model.images.values():
        if image.has_pose:
            pose = np.eye(4)
            pose[:3, :4] = image.cam_from_world().matrix()
            poses[image.name] = pose
    camera = next(iter(model.cameras.values()))
    fx, fy, cx, cy, k1, k2, k3, k4 = (float(v) for v in camera.params)
    error = float(model.compute_mean_reprojection_error())
    lens = Lens(fx, fy, cx, cy, (k1, k2, k3, k4), width, height, error, len(names), len(poses))
    return poses, lens


def _ahead(times: Sequence[float], t: float, window: float) -> float | None:
    """Return the time nearest ``t + window`` among sorted `times`, if within a quarter window."""
    k = bisect.bisect_left(times, t + window)
    near = [times[j] for j in (k - 1, k) if 0 <= j < len(times)]
    best = min(near, key=lambda u: abs(u - (t + window)), default=None)
    if best is None or best <= t or abs(best - (t + window)) > window / 4:
        return None
    return best


def _turns(poses: Poses, window: float) -> tuple[Array, Array]:
    """Return each time with a pose a window later, and the camera's turn over it, in radians."""
    times = sorted(poses)
    out_t, out_r = [], []
    for t in times:
        ahead = _ahead(times, t, window)
        if ahead is not None:
            turn = poses[ahead][:3, :3] @ poses[t][:3, :3].T
            out_t.append(t)
            out_r.append(float(np.linalg.norm(so3.log(turn))))
    return np.array(out_t), np.array(out_r)


def time_offset(
    front: Poses, rear: Poses, *, window: float = WINDOW, search: float = OFFSET_SEARCH
) -> tuple[float, float]:
    """Return the offset that best lines the rear's clock up with the front's, and its correlation.

    The offset ``d`` is such that the rear's time ``t + d`` is the front's ``t``: there the
    rear's turn over each window, interpolated at the front's times shifted by ``d``, correlates
    best with the front's (Pearson).

    Parameters
    ----------
    front, rear : mapping of float to ndarray, shape (4, 4)
        Each camera's ``T_cam_world`` by time.
    window : float, optional
        The time each turn spans, in seconds.
    search : float, optional
        How far either way to search, in seconds.

    Returns
    -------
    tuple of float
        The offset, in seconds, and its correlation.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If either camera has fewer than :data:`MIN_RATES` turns, or none of the offsets
        overlaps them enough to correlate.
    """
    ft, fr = _turns(front, window)
    rt, rr = _turns(rear, window)
    if len(ft) < MIN_RATES or len(rt) < MIN_RATES:
        raise InvalidInputError("front, rear: too few poses a window apart to correlate")

    def correlation(d: float) -> float:
        shifted = ft + d
        inside = (shifted >= rt[0]) & (shifted <= rt[-1])
        if int(inside.sum()) < MIN_RATES:
            return -math.inf
        a, b = fr[inside], np.interp(shifted[inside], rt, rr)
        if a.std() == 0.0 or b.std() == 0.0:
            return -math.inf
        return float(np.corrcoef(a, b)[0, 1])

    coarse = [float(d) for d in np.arange(-search, search + 1e-9, OFFSET_COARSE)]
    best = max(coarse, key=correlation)
    fine = [float(d) for d in np.arange(-OFFSET_COARSE, OFFSET_COARSE + 1e-9, OFFSET_STEP)]
    offset = max((best + d for d in fine), key=correlation)
    value = correlation(offset)
    if not math.isfinite(value):
        raise InvalidInputError("front, rear: no offset overlaps their turns enough")
    return offset, value


def _interpolate(poses: Poses, times: Sequence[float], at: float) -> Array | None:
    """Return the pose at a time between two posed frames: rotation by slerp, centre linearly.

    None outside the poses, or across a gap in them (a frame not placed).
    """
    k = bisect.bisect_left(times, at)
    if k < len(times) and times[k] == at:
        return np.asarray(poses[at])
    if k == 0 or k >= len(times):
        return None
    t0, t1 = times[k - 1], times[k]
    if t1 - t0 > 1.5 * float(np.median(np.diff(times))):
        return None
    a, b = poses[t0], poses[t1]
    alpha = (at - t0) / (t1 - t0)
    rotation = a[:3, :3] @ so3.exp(alpha * so3.log(a[:3, :3].T @ b[:3, :3]))
    centre = (1 - alpha) * (-a[:3, :3].T @ a[:3, 3]) + alpha * (-b[:3, :3].T @ b[:3, 3])
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = -rotation @ centre
    return pose


def motion_pairs(
    front: Poses, rear: Poses, offset: float, window: float = WINDOW
) -> list[tuple[Array, Array]]:
    """Return the two cameras' relative motions over the same windows, the rear's clock shifted.

    Each pair is ``(A, B)``: the front's ``T(t + w) T(t)^-1`` and the rear's over the same
    window shifted by `offset`, interpolated between its frames.

    Parameters
    ----------
    front, rear : mapping of float to ndarray, shape (4, 4)
        Each camera's ``T_cam_world`` by time.
    offset : float
        The rear's clock offset (:func:`time_offset`).
    window : float, optional
        The time each motion spans, in seconds.

    Returns
    -------
    list of tuple of ndarray
        The pairs, in time order.
    """
    front_times = sorted(front)
    rear_times = sorted(rear)
    pairs = []
    for t in front_times:
        ahead = _ahead(front_times, t, window)
        if ahead is None:
            continue
        b0 = _interpolate(rear, rear_times, t + offset)
        b1 = _interpolate(rear, rear_times, ahead + offset)
        if b0 is None or b1 is None:
            continue
        pairs.append((front[ahead] @ np.linalg.inv(front[t]), b1 @ np.linalg.inv(b0)))
    return pairs


@dataclass(frozen=True)
class RigRotation:
    """The rig's rotation, and how well each correspondence fits it.

    Attributes
    ----------
    rotation : ndarray of float64, shape (3, 3)
        ``R_front_rear``.
    turns : ndarray of float64, shape (T,)
        Each turning pair's residual ``|log(R_A R_X (R_X R_B)^T)|``, in radians.
    directions : ndarray of float64, shape (D,)
        Each straight pair's angle between the front's direction of travel and the rear's
        turned by ``R_X``, in radians.
    """

    rotation: Array
    turns: Array
    directions: Array


def _unit(vectors: Array) -> Array:
    """Return each row scaled to unit length."""
    return np.asarray(vectors / np.linalg.norm(vectors, axis=1, keepdims=True), dtype=np.float64)


def _angles(a: Array, b: Array) -> Array:
    """Return the angle between each row of `a` and of `b`, unit vectors, in radians."""
    return np.asarray(np.arccos(np.clip(np.sum(a * b, axis=1), -1.0, 1.0)), dtype=np.float64)


def hand_eye_rotation(
    pairs: Sequence[tuple[Array, Array]], min_turn: float = MIN_TURN
) -> RigRotation:
    """Return the rig's rotation ``R_front_rear`` from paired motions (the module docstring).

    Two kinds of pair give a direction in each camera's frame that ``R_X`` turns the rear's onto
    the front's: a pair that turns at least `min_turn` gives its rotation axis (``a = R_X b``),
    and a pair that turns less but moves (at least a fifth of the median pair's distance) gives
    its direction of travel, the car's forward (``-t_A`` and ``-t_B``, the same displacement in
    each camera's frame). A drive turns about the vertical almost only, so its axes alone leave
    the rig's rotation about the vertical open; the directions of travel close it, as
    camera-to-odometry calibration closes it from the translations (Guo et al. 2012). Kabsch's
    least squares fits ``R_X`` to both, each a unit vector.

    Parameters
    ----------
    pairs : sequence of tuple of ndarray
        ``(A, B)`` relative motions (:func:`motion_pairs`).
    min_turn : float, optional
        The least turn, in radians, of a pair's front motion for its axis to count.

    Returns
    -------
    RigRotation
        The rotation, and the residuals.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If fewer than 3 pairs turn enough, or none moves straight.
    """
    if not pairs:
        raise InvalidInputError("pairs: none given")
    rot_a = np.array([a[:3, :3] for a, _ in pairs])
    rot_b = np.array([b[:3, :3] for _, b in pairs])
    log_a = so3.log(rot_a)
    turning = np.linalg.norm(log_a, axis=1) > min_turn
    moved_a = -np.array([a[:3, 3] for a, _ in pairs])
    moved_b = -np.array([b[:3, 3] for _, b in pairs])
    distance = np.linalg.norm(moved_a, axis=1)
    straight = ~turning & (distance > 0.2 * float(np.median(distance)))
    straight &= np.linalg.norm(moved_b, axis=1) > 0.0
    if int(turning.sum()) < 3 or not straight.any():
        raise InvalidInputError("pairs: expected 3 that turn and 1 that moves straight")
    a = np.vstack([_unit(log_a[turning]), _unit(moved_a[straight])])
    b = np.vstack([_unit(so3.log(rot_b[turning])), _unit(moved_b[straight])])
    u, _, vt = np.linalg.svd(b.T @ a)
    d = float(np.sign(np.linalg.det(vt.T @ u.T))) or 1.0
    rotation = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    misfit = rot_a[turning] @ rotation @ np.swapaxes(rotation @ rot_b[turning], 1, 2)
    turns = np.linalg.norm(so3.log(misfit), axis=1)
    directions = _angles(_unit(moved_a[straight]), _unit(moved_b[straight]) @ rotation.T)
    return RigRotation(rotation, turns, directions)


def gps_scale(front: Poses, track: gnss.Track, window: float = WINDOW) -> tuple[float, int]:
    """Return metres per unit of the front camera's map, from the GPS track's speed.

    Over each window the car moves at least :data:`MIN_SPEED`, the track's distance is divided
    by the map's; the median of those is the scale.

    Parameters
    ----------
    front : mapping of float to ndarray, shape (4, 4)
        The front camera's ``T_cam_world`` by time, on the track's clock.
    track : slam_it.eval.gnss.Track
        The smoothed GPS track.
    window : float, optional
        The time each distance spans, in seconds.

    Returns
    -------
    tuple
        Metres per map unit, and the windows it is the median of.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If fewer than 5 windows lie within the track at speed.
    """
    times = sorted(front)
    ratios = []
    for t in times:
        ahead = _ahead(times, t, window)
        if ahead is None or t < track.times[0] or ahead > track.times[-1]:
            continue
        here, there = track.at([t, ahead])
        metres = float(np.linalg.norm(there - here))
        a, b = front[t], front[ahead]
        units = float(np.linalg.norm(a[:3, :3].T @ a[:3, 3] - b[:3, :3].T @ b[:3, 3]))
        if metres / (ahead - t) >= MIN_SPEED and units > 0.0:
            ratios.append(metres / units)
    if len(ratios) < 5:
        raise InvalidInputError("track: fewer than 5 windows at speed within it to scale the map")
    return float(np.median(ratios)), len(ratios)


@dataclass(frozen=True)
class Translation:
    """The rig's translation and the rear map's scale, with their uncertainty.

    Attributes
    ----------
    t : ndarray of float64, shape (3,)
        The rear camera's centre in the front camera's frame, in metres (``x`` right, ``y``
        down, ``z`` forward).
    rear_scale : float
        Metres per unit of the rear camera's map.
    covariance : ndarray of float64, shape (4, 4)
        The posterior covariance of ``(t, rear_scale)``.
    residual : float
        The motions' root-mean-square residual, in metres.
    pairs : int
        The pairs used.
    """

    t: Array
    rear_scale: float
    covariance: Array
    residual: float
    pairs: int


def rig_translation(
    pairs: Sequence[tuple[Array, Array]],
    rotation: Array,
    *,
    front_scale: float,
    prior: Array,
    prior_sigma: Array,
) -> Translation:
    """Return the rig's translation and the rear map's scale, maximum a posteriori.

    Each pair gives ``(R_A - I) t - s R_X t_B = -front_scale t_A``, linear in ``(t, s)``. The
    motions' noise is taken from an unweighted least-squares fit's residual, then the Gaussian
    prior on ``t`` joins them; the problem is linear-Gaussian, so the estimate and its
    covariance are closed form.

    Parameters
    ----------
    pairs : sequence of tuple of ndarray
        ``(A, B)`` relative motions (:func:`motion_pairs`), each in its map's units.
    rotation : ndarray, shape (3, 3)
        ``R_front_rear`` (:func:`hand_eye_rotation`).
    front_scale : float
        Metres per unit of the front map (:func:`gps_scale`).
    prior : ndarray, shape (3,)
        The prior's mean for ``t``, in metres: the measured baseline.
    prior_sigma : ndarray, shape (3,)
        Its standard deviation per axis, in metres.

    Returns
    -------
    Translation
        The estimate and its covariance.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If there are fewer than 3 pairs, or the scale or a sigma is not positive.
    """
    mean = np.asarray(prior, dtype=np.float64)
    sigma = np.asarray(prior_sigma, dtype=np.float64)
    if len(pairs) < 3:
        raise InvalidInputError("pairs: expected at least 3")
    if not front_scale > 0.0 or mean.shape != (3,) or sigma.shape != (3,) or np.any(sigma <= 0):
        raise InvalidInputError("front_scale, prior, prior_sigma: expected positive values, (3,)")
    rows = [np.hstack([a[:3, :3] - np.eye(3), (-rotation @ b[:3, 3])[:, None]]) for a, b in pairs]
    j = np.vstack(rows)
    y = np.hstack([-front_scale * a[:3, 3] for a, _ in pairs])
    first, *_ = np.linalg.lstsq(j, y, rcond=None)
    noise = max(float(np.sqrt(np.mean((j @ first - y) ** 2))), 1e-6)
    information = j.T @ j / noise**2
    information[:3, :3] += np.diag(1.0 / sigma**2)
    vector = j.T @ y / noise**2
    vector[:3] += mean / sigma**2
    covariance = np.asarray(np.linalg.inv(information), dtype=np.float64)
    estimate = covariance @ vector
    residual = float(np.sqrt(np.mean((j @ estimate - y) ** 2)))
    return Translation(estimate[:3], float(estimate[3]), covariance, residual, len(pairs))


@dataclass(frozen=True)
class Report:
    """A rig calibration: each lens, the clocks, the rig, and how well each fits.

    Attributes
    ----------
    front, rear : Lens
        Each camera's lens.
    offset, correlation : float
        The rear clock's offset (:func:`time_offset`), in seconds, and its correlation.
    rotation : RigRotation
        ``R_front_rear`` and its residuals.
    front_scale : float
        Metres per front map unit (:func:`gps_scale`).
    scale_windows : int
        The windows the front scale is the median of.
    translation : Translation
        The rig's translation and the rear map's scale.
    """

    front: Lens
    rear: Lens
    offset: float
    correlation: float
    rotation: RigRotation
    front_scale: float
    scale_windows: int
    translation: Translation

    def as_dict(self, front_size: tuple[int, int], rear_size: tuple[int, int]) -> dict[str, Any]:
        """Return the report as plain values for JSON, each lens at its video's size.

        Parameters
        ----------
        front_size, rear_size : tuple of int
            Each camera's video size, ``(width, height)``.

        Returns
        -------
        dict
            The report.
        """

        def lens(found: Lens, size: tuple[int, int]) -> dict[str, Any]:
            at = found.fisheye(*size)
            return {
                "model": "fisheye",
                "size": list(size),
                "fx": at.camera.fx,
                "fy": at.camera.fy,
                "cx": at.camera.cx,
                "cy": at.camera.cy,
                "k": list(at.k),
                "aspect": found.aspect,
                "square_pixels_fit": abs(found.aspect - 1.0) <= ASPECT_TOLERANCE,
                "reprojection_error_px": found.error,
                "reconstructed_size": [found.width, found.height],
                "frames": found.frames,
                "registered": found.registered,
            }

        def spread(radians: Array) -> dict[str, Any]:
            degrees = np.degrees(radians)
            return {
                "count": len(degrees),
                "median": float(np.median(degrees)),
                "p90": float(np.percentile(degrees, 90)),
            }

        rotation = self.rotation.rotation
        sd = np.sqrt(np.diag(self.translation.covariance))
        return {
            "front": lens(self.front, front_size),
            "rear": lens(self.rear, rear_size),
            "rear_clock_offset_s": self.offset,
            "offset_correlation": self.correlation,
            "rotation_front_rear": rotation.tolist(),
            "rotation_angle_deg": float(np.degrees(np.linalg.norm(so3.log(rotation)))),
            "turn_residual_deg": spread(self.rotation.turns),
            "direction_residual_deg": spread(self.rotation.directions),
            "front_metres_per_unit": self.front_scale,
            "front_scale_windows": self.scale_windows,
            "translation_front_rear_m": self.translation.t.tolist(),
            "translation_sd_m": sd[:3].tolist(),
            "baseline_m": float(np.linalg.norm(self.translation.t)),
            "rear_metres_per_unit": self.translation.rear_scale,
            "rear_scale_sd": float(sd[3]),
            "translation_residual_m": self.translation.residual,
            "translation_pairs": self.translation.pairs,
        }


def by_time(poses: Mapping[str, Array], clocks: Mapping[str, float]) -> dict[float, Array]:
    """Return poses keyed by their frames' times (:func:`extract_frames`) for their file names."""
    return {clocks[name]: pose for name, pose in poses.items()}


def solve(
    front: Poses,
    rear: Poses,
    found: tuple[Lens, Lens],
    track: gnss.Track,
    *,
    baseline: float,
) -> Report:
    """Calibrate the rig from the two cameras' reconstructions (the module docstring's 2 to 4).

    Parameters
    ----------
    front, rear : mapping of float to ndarray, shape (4, 4)
        Each camera's ``T_cam_world`` by time (:func:`reconstruct`, :func:`by_time`).
    found : tuple of Lens
        The front's lens and the rear's.
    track : slam_it.eval.gnss.Track
        The smoothed GPS track, on the front's clock.
    baseline : float
        The measured distance from the front camera to the rear, in metres: the prior's mean
        is that far behind the front camera.

    Returns
    -------
    Report
        The calibration.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the baseline is not positive, or a step finds too little to work from.
    """
    if not baseline > 0.0:
        raise InvalidInputError(f"baseline: expected a positive distance, got {baseline}")
    offset, correlation = time_offset(front, rear)
    pairs = motion_pairs(front, rear, offset)
    rotation = hand_eye_rotation(pairs)
    front_scale, windows = gps_scale(front, track)
    translation = rig_translation(
        pairs,
        rotation.rotation,
        front_scale=front_scale,
        prior=np.array([0.0, 0.0, -baseline]),
        prior_sigma=np.array(PRIOR_SIGMA),
    )
    return Report(
        found[0],
        found[1],
        offset,
        correlation,
        rotation,
        front_scale,
        windows,
        translation,
    )
