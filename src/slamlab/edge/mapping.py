"""Local mapping: grow and clean the map around each new keyframe (ORB-SLAM's ``LocalMapping``).

After tracking decides a frame becomes a keyframe (:func:`need_keyframe`), :func:`process`
runs ORB-SLAM's local mapping steps in order, single-threaded:

1. **Insert the keyframe** with the points it tracked; a stereo or RGB-D keyframe also
   creates points from its close depth (:func:`create_points_from_depth`,
   ``CreateNewKeyFrame``).
2. **Cull new points** (:func:`cull_points`, ``MapPointCulling``).
3. **Triangulate new points** between the keyframe and the window's others
   (:func:`triangulate_new_points`, ``CreateNewMapPoints``).
4. **Fuse duplicates** (:func:`fuse_points`, ``SearchInNeighbors``).
5. **Local BA** over the window (:func:`slamlab.edge.adjust.bundle_adjust`), the map's first
   keyframe fixed, once the map holds ``ba_min_keyframes`` (``KeyFramesInMap() > 2``).
6. **Cull redundant keyframes** (:func:`cull_keyframes`, ``KeyFrameCulling``).

Lite: the sliding window of the last ``window`` keyframes stands in for the covisibility
graph wherever ORB-SLAM takes a keyframe's neighbours, and the newest keyframe for the
reference keyframe. Matching for triangulation has no bag of words: every free keypoint pair
within the stricter distance and the epipolar band is a candidate
(:func:`match_for_triangulation`).
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.edge import adjust, residuals
from slamlab.edge import map as maps
from slamlab.edge.frames import Camera, Frame, Tracked, as_pose, checked_tracked
from slamlab.edge.map import Map
from slamlab.edge.options import Options, check_mode
from slamlab.foundations import matching, triangulation
from slamlab.foundations.arrays import as_real
from slamlab.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


Crossing = dict[str, Any]
"""A map, camera, tracked frame or the settings, as it crosses the bindings."""


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.mapping`` submodule provides.

    The map crosses as :meth:`Map.as_dict`, a frame as :meth:`slamlab.edge.frames.Frame.as_dict`, a
    tracked frame as :meth:`Tracked.as_dict`, the mode by name.
    """

    def need_keyframe(  # noqa: PLR0913 -- the stage's state, as the reference takes it
        self,
        *,
        m: Crossing,
        camera: Crossing,
        depth: NDArray[np.float64],
        tracked: Crossing,
        mode: str,
        options: Crossing,
    ) -> bool:
        """Decide whether a tracked frame becomes a keyframe."""
        ...

    def create_points_from_depth(
        self, m: Crossing, camera: Crossing, options: Crossing
    ) -> Crossing:
        """Create points from the newest keyframe's close depth."""
        ...

    def cull_points(self, m: Crossing, mode: str, options: Crossing) -> Crossing:
        """Cull recently created points tracking rarely finds."""
        ...

    def match_for_triangulation(
        self, m: Crossing, camera: Crossing, k1: int, k2: int, options: Crossing
    ) -> NDArray[np.int64]:
        """Match two keyframes' free keypoints for triangulation."""
        ...

    def triangulate_new_points(
        self, m: Crossing, camera: Crossing, mode: str, options: Crossing
    ) -> Crossing:
        """Triangulate the newest keyframe's free keypoints against the window's."""
        ...

    def fuse_points(self, m: Crossing, camera: Crossing, options: Crossing) -> Crossing:
        """Fuse the newest keyframe's points with the window's."""
        ...

    def cull_keyframes(
        self, m: Crossing, camera: Crossing, mode: str, options: Crossing
    ) -> Crossing:
        """Cull the window's redundant keyframes."""
        ...

    def process(  # noqa: PLR0913 -- the stage's state, as the reference takes it
        self,
        *,
        m: Crossing,
        camera: Crossing,
        frame: Crossing,
        tracked: Crossing,
        mode: str,
        options: Crossing,
    ) -> Crossing:
        """Insert a keyframe and run local mapping on it."""
        ...


def _arguments(m: Map, camera: Camera | None, options: Options, **items: object) -> Crossing:
    """Return a compiled step's arguments: the map, the camera, the settings, and the rest."""
    out: Crossing = {"m": m.as_dict(), "options": options.as_dict(), **items}
    if camera is not None:
        out["camera"] = camera.as_dict()
    return out


def _check_newest(m: Map) -> None:
    """Refuse a map without keyframes (the steps work on its newest)."""
    if m.keyframes == 0:
        raise InvalidInputError("m: expected a map with at least one keyframe")


def close_depth(camera: Camera, options: Options) -> float:
    """Return the depth below which a point is close: ``close_depth`` baselines (``mThDepth``).

    Parameters
    ----------
    camera : slamlab.edge.frames.Camera
        The camera (its baseline).
    options : Options
        The factor.

    Returns
    -------
    float
        The depth, in metres (0 for monocular).
    """
    return options.close_depth * camera.baseline


def _world(pose: Array, local: Array) -> Array:
    """Move camera-frame points into the world: ``R^T (X - t)``."""
    return np.asarray((local - pose[:3, 3]) @ pose[:3, :3], dtype=np.float64)


def _keypoint_indices(m: Map, k: int) -> IntArray:
    """Return a keyframe's keypoint rows."""
    return np.arange(m.kf_start[k], m.kf_start[k + 1], dtype=np.int64)


def _from_depth(m: Map, k: int, rows: IntArray) -> Array:
    """Return the world points keypoints' depths give (``UnprojectStereo``)."""
    bearings = m.kp_bearing[rows]
    return _world(m.kf_pose[k], bearings * (m.kp_depth[rows] / bearings[:, 2])[:, None])


def _need_keyframe(  # noqa: PLR0913 -- the stage's state
    m: Map, camera: Camera, depth: Array, tracked: Tracked, *, mode: str, options: Options
) -> bool:
    """Decide whether a tracked frame becomes a keyframe (reference)."""
    early = m.keyframes <= 2  # NeedNewKeyFrame's nKFs <= 2
    min_observations = options.ref_min_observations_early if early else options.ref_min_observations
    seen = m.kp_point[m.keypoint_rows(m.keyframes - 1)]
    rows = m.point_rows(seen[seen >= 0])
    weights = maps.observation_weight(m, rows[rows >= 0])
    tracked_reference = int(np.count_nonzero(weights >= min_observations))
    need_close = False
    if mode != "monocular":
        close = (depth > 0.0) & (depth < close_depth(camera, options))
        tracked_close = int(np.count_nonzero(close & (tracked.points >= 0)))
        untracked_close = int(np.count_nonzero(close & (tracked.points < 0)))
        need_close = (
            tracked_close < options.close_tracked and untracked_close > options.close_untracked
        )
    ratio = options.first_ref_ratio if m.keyframes < 2 else options.ref_ratio
    if mode == "monocular":
        ratio = options.mono_ref_ratio
    weak = tracked.inliers < tracked_reference * ratio or need_close
    return bool(weak and tracked.inliers > options.min_keyframe_inliers)


def need_keyframe(  # noqa: PLR0913 -- the stage's state, and the backend
    m: Map,
    camera: Camera,
    depth: ArrayLike,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    backend: str = "python",
) -> bool:
    """Decide whether a tracked frame becomes a keyframe (``NeedNewKeyFrame``, single-threaded).

    With mapping never busy, ORB-SLAM's conditions reduce to: the frame tracks fewer than
    ``ref_ratio`` (``mono_ref_ratio``; ``first_ref_ratio`` with one keyframe) of the
    points the reference keyframe (here the newest) sees from two keyframes (three once
    there are more than two), or, with depth, fewer than ``close_tracked`` close points while
    more than ``close_untracked`` close ones go untracked; and it tracks more than
    ``min_keyframe_inliers``.

    Parameters
    ----------
    m : Map
        The map, with at least one keyframe.
    camera : slamlab.edge.frames.Camera
        The camera.
    depth : array_like, shape (N,)
        The frame's keypoints' depths (NaN where unknown).
    tracked : slamlab.edge.frames.Tracked
        Its pose and matches.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The ratios and counts.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    bool
        Whether to insert a keyframe.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the mode is unknown, the map has no keyframe, the depths are infinite, or the
        matches are not one per depth.
    """
    check_mode(mode)
    _check_newest(m)
    depths = as_real(depth, "depth", allow_nan=True).reshape(-1)
    ids = np.asarray(tracked.points)
    if ids.shape != depths.shape or not np.issubdtype(ids.dtype, np.integer):
        raise InvalidInputError("tracked.points: expected one point id (or -1) per depth")
    checked = Tracked(pose=as_pose(tracked.pose, "tracked.pose"), points=ids.astype(np.int64))
    return backends.call(
        backend,
        __name__,
        "need_keyframe",
        reference=lambda: _need_keyframe(m, camera, depths, checked, mode=mode, options=options),
        arguments=_arguments(
            m, camera, options, depth=depths, tracked=checked.as_dict(), mode=mode
        ),
        decode=lambda raw, codes: bool(raw),
    )


def _create_points_from_depth(m: Map, camera: Camera, options: Options) -> Map:
    """Create points from the newest keyframe's close depth (reference)."""
    k = m.keyframes - 1
    rows = _keypoint_indices(m, k)
    depth = m.kp_depth[rows]
    usable = (depth > 0.0) & (m.kp_bearing[rows, 2] > 0.0)
    order = rows[usable][np.argsort(depth[usable], kind="stable")]
    limit = close_depth(camera, options)
    chosen = []
    for count, row in enumerate(order, start=1):
        if m.kp_point[row] < 0:
            chosen.append(row)
        if m.kp_depth[row] > limit and count > options.min_new_points:
            break
    if not chosen:
        return m
    new = np.array(chosen, dtype=np.int64)
    return maps.add_points(m, _from_depth(m, k, new), new[:, None])


def create_points_from_depth(
    m: Map, camera: Camera, options: Options, *, backend: str = "python"
) -> Map:
    """Create points from the newest keyframe's close depth (``CreateNewKeyFrame``).

    Keypoints with depth, nearest first: each one not observing a point creates one, until
    the keyframe holds at least ``min_new_points`` (counting those it tracked) and the next
    is no longer close.

    Parameters
    ----------
    m : Map
        The map, its newest keyframe stereo or RGB-D.
    camera : slamlab.edge.frames.Camera
        The camera.
    options : Options
        The counts and close depth.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the new points.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the map has no keyframe.
    """
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "create_points_from_depth",
        reference=lambda: _create_points_from_depth(m, camera, options),
        arguments=_arguments(m, camera, options),
        decode=maps.decode,
    )


def _cull_points(m: Map, mode: str, options: Options) -> Map:
    """Cull recently created points tracking rarely finds (reference)."""
    age = m.kf_id[-1] - m.pt_first_kf
    recent = np.flatnonzero(age <= options.recent_point_keyframes)
    if len(recent) == 0:
        return m
    minimum = (
        options.min_point_observations
        if mode == "monocular"
        else options.min_point_observations_depth
    )
    ratio = m.pt_found[recent] / m.pt_visible[recent]
    weight = maps.observation_weight(m, recent)
    erase = np.zeros(m.points, dtype=bool)
    erase[recent] = (ratio < options.found_ratio) | (
        (age[recent] >= options.point_cull_age) & (weight <= minimum)
    )
    return maps.erase_points(m, erase)


def cull_points(m: Map, mode: str, options: Options, *, backend: str = "python") -> Map:
    """Cull recently created points that tracking rarely finds (``MapPointCulling``).

    A point created at most ``recent_point_keyframes`` keyframes ago goes when found in under
    ``found_ratio`` of the frames that had it in view, or when, two or more keyframes after
    its creation, it is observed with weight ``min_point_observations`` or less
    (``min_point_observations_depth`` with depth).

    Parameters
    ----------
    m : Map
        The map, its newest keyframe the current one.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The ratio and minimums.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map without the culled points.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the mode is unknown, or the map has no keyframe.
    """
    check_mode(mode)
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "cull_points",
        reference=lambda: _cull_points(m, mode, options),
        arguments=_arguments(m, None, options, mode=mode),
        decode=maps.decode,
    )


def _relative(m: Map, k1: int, k2: int) -> tuple[Array, Array]:
    """Return ``T_2_1``'s rotation and translation: ``R2 R1^T`` and ``t2 - R t1``."""
    pose1, pose2 = m.kf_pose[k1], m.kf_pose[k2]
    rotation = pose2[:3, :3] @ pose1[:3, :3].T
    return rotation, pose2[:3, 3] - rotation @ pose1[:3, 3]


def _free_rows(m: Map, k: int) -> IntArray:
    """Return a keyframe's keypoints observing no point, with a bearing in front."""
    rows = _keypoint_indices(m, k)
    free: IntArray = rows[(m.kp_point[rows] < 0) & (m.kp_bearing[rows, 2] > 0.0)]
    return free


def _match_for_triangulation(
    m: Map, camera: Camera, k1: int, k2: int, options: Options
) -> IntArray:
    """Match two keyframes' free keypoints for triangulation (reference)."""
    rows1, rows2 = _free_rows(m, k1), _free_rows(m, k2)
    if len(rows1) == 0 or len(rows2) == 0:
        return np.zeros((0, 2), dtype=np.int64)
    rotation, t = _relative(m, k1, k2)
    skew = np.array([[0.0, -t[2], t[1]], [t[2], 0.0, -t[0]], [-t[1], t[0], 0.0]])
    essential = skew @ rotation
    x1 = m.kp_bearing[rows1] / m.kp_bearing[rows1, 2:]
    x2 = m.kp_bearing[rows2] / m.kp_bearing[rows2, 2:]
    lines = x1 @ essential.T  # each k1 keypoint's epipolar line in k2, normalised plane
    numerator = lines @ x2.T
    distance2 = numerator**2 / (lines[:, :1] ** 2 + lines[:, 1:2] ** 2) * camera.focal**2
    band = distance2 < options.epipolar_chi2 * residuals.sigma2(m, m.kp_level[rows2])[None, :]
    if t[2] != 0.0:  # camera 1's centre in camera 2 is t: the epipole
        epipole = t[:2] / t[2]
        near = np.sum((x2[:, :2] - epipole) ** 2, axis=1) * camera.focal**2 < (
            options.min_epipole_distance2 * m.scale_factor ** m.kp_level[rows2].astype(np.float64)
        )
        no_depth = np.isnan(m.kp_depth[rows1])[:, None] & np.isnan(m.kp_depth[rows2])[None, :]
        band &= ~(no_depth & near[None, :])
    hamming = matching.distances(m.kp_descriptor[rows1], m.kp_descriptor[rows2])
    hamming = np.where(band & (hamming <= options.th_low), hamming, np.iinfo(np.int64).max)
    best = np.argmin(hamming, axis=1)
    best_distance = hamming[np.arange(len(rows1)), best]
    claims: dict[int, tuple[int, int]] = {}
    for i in np.flatnonzero(best_distance <= options.th_low):
        j, d = int(best[i]), int(best_distance[i])
        if j not in claims or d < claims[j][0]:
            claims[j] = (d, int(i))
    pairs = sorted((i, j) for j, (_, i) in claims.items())
    return np.array([[rows1[i], rows2[j]] for i, j in pairs], dtype=np.int64).reshape(-1, 2)


def match_for_triangulation(  # noqa: PLR0913 -- the map, its camera, two keyframes, the settings
    m: Map, camera: Camera, k1: int, k2: int, options: Options, *, backend: str = "python"
) -> IntArray:
    """Match two keyframes' free keypoints for triangulation (``SearchForTriangulation``, lite).

    No bag of words: for each free keypoint of `k1` (one observing no point), the nearest
    free keypoint of `k2` within ``th_low`` (ties: the first) that lies within the epipolar
    band (``epipolar_chi2`` of its level's variance, in pixels at the focal length) and,
    when neither has depth, not near the epipole (``min_epipole_distance2``). One match per
    keypoint of `k2`: the nearest keeps it (ties: the lower `k1` keypoint).

    Parameters
    ----------
    m : Map
        The map.
    camera : slamlab.edge.frames.Camera
        The camera.
    k1, k2 : int
        The two keyframes' rows.
    options : Options
        The epipolar band and the epipole's distance.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (Q, 2)
        Keypoint rows ``(in k1, in k2)``, by `k1` keypoint.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a keyframe row is out of range, or the two are the same.
    """
    first = maps.as_keyframe_row(m, k1, "k1")
    second = maps.as_keyframe_row(m, k2, "k2")
    if first == second:
        raise InvalidInputError("k1, k2: expected two different keyframes")
    return backends.call(
        backend,
        __name__,
        "match_for_triangulation",
        reference=lambda: _match_for_triangulation(m, camera, first, second, options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "k1": first,
            "k2": second,
            "options": options.as_dict(),
        },
        decode=lambda raw, codes: backends.checked_array(raw, np.int64, 2, "triangulation pairs"),
    )


def _stereo_cos(m: Map, camera: Camera, rows: IntArray) -> Array:
    """Return the parallax cosine keypoints' depths give: ``cos(2 atan2(b / 2, z))``."""
    return np.asarray(np.cos(2.0 * np.arctan2(camera.baseline / 2.0, m.kp_depth[rows])))


def _triangulate(  # noqa: PLR0913 -- the two keyframes, their matches, the settings
    m: Map, camera: Camera, k1: int, k2: int, pairs: IntArray, *, options: Options
) -> tuple[Array, Mask]:
    """Triangulate matched keypoints; return the points and which pass every check."""
    rows1, rows2 = pairs[:, 0], pairs[:, 1]
    rays1 = m.kp_bearing[rows1] @ m.kf_pose[k1][:3, :3]  # world directions
    rays2 = m.kp_bearing[rows2] @ m.kf_pose[k2][:3, :3]
    cos_rays = np.sum(rays1 * rays2, axis=1)
    has1, has2 = ~np.isnan(m.kp_depth[rows1]), ~np.isnan(m.kp_depth[rows2])
    # ORB-SLAM takes the depth parallax of the first keypoint with depth only; the other's
    # stands at cos_rays + 1, above any ray parallax.
    cos1 = np.where(has1, _stereo_cos(m, camera, rows1), cos_rays + 1.0)
    cos2 = np.where(~has1 & has2, _stereo_cos(m, camera, rows2), cos_rays + 1.0)
    cos_stereo = np.minimum(cos1, cos2)
    from_rays = (
        (cos_rays < cos_stereo)
        & (cos_rays > 0.0)
        & (has1 | has2 | (cos_rays < options.max_parallax_cos))
    )
    from_depth1 = ~from_rays & has1 & (cos1 < cos2)
    from_depth2 = ~from_rays & ~from_depth1 & has2 & (cos2 < cos1)
    xyz = np.full((len(pairs), 3), np.nan)
    if np.any(from_rays):
        rotation, t = _relative(m, k1, k2)
        relative = np.eye(4)
        relative[:3, :3], relative[:3, 3] = rotation, t
        found = triangulation.two_view(
            relative, m.kp_bearing[rows1[from_rays]], m.kp_bearing[rows2[from_rays]]
        )
        xyz[from_rays] = _world(m.kf_pose[k1], found.points)
    if np.any(from_depth1):
        xyz[from_depth1] = _from_depth(m, k1, rows1[from_depth1])
    if np.any(from_depth2):
        xyz[from_depth2] = _from_depth(m, k2, rows2[from_depth2])
    keep = ~np.isnan(xyz[:, 0])
    filled = np.where(keep[:, None], xyz, 0.0)
    for k, rows in ((k1, rows1), (k2, rows2)):
        error = residuals.chi2(
            camera,
            maps.to_camera(m.kf_pose[k], filled),
            m.kp_xy[rows],
            m.kp_right[rows],
            residuals.sigma2(m, m.kp_level[rows]),
        )
        keep &= error < residuals.thresholds(m.kp_right[rows], options)
    centres = m.centres()
    distance1 = np.linalg.norm(filled - centres[k1], axis=1)
    distance2 = np.linalg.norm(filled - centres[k2], axis=1)
    keep &= (distance1 > 0.0) & (distance2 > 0.0)
    ratio_distance = distance2 / np.where(distance1 > 0.0, distance1, 1.0)
    ratio_octave = m.scale_factor ** (m.kp_level[rows1] - m.kp_level[rows2]).astype(np.float64)
    factor = options.scale_ratio * m.scale_factor
    keep &= ~((ratio_distance * factor < ratio_octave) | (ratio_distance > ratio_octave * factor))
    return filled, keep


def _median_depth(m: Map, k: int) -> float | None:
    """Return a keyframe's median scene depth, the lower middle (``ComputeSceneMedianDepth``)."""
    seen = m.kp_point[m.keypoint_rows(k)]
    rows = m.point_rows(seen[seen >= 0])
    rows = rows[rows >= 0]
    if len(rows) == 0:
        return None
    depths = np.sort(maps.to_camera(m.kf_pose[k], m.pt_xyz[rows])[:, 2])
    return float(depths[(len(depths) - 1) // 2])


def _triangulate_new_points(m: Map, camera: Camera, mode: str, options: Options) -> Map:
    """Triangulate the newest keyframe's free keypoints against the window's (reference)."""
    k1 = m.keyframes - 1
    neighbours = (
        options.mono_triangulation_neighbours
        if mode == "monocular"
        else options.triangulation_neighbours
    )
    others = m.window(options.window)[:-1][::-1][:neighbours]
    centres = m.centres()
    for k2 in (int(k) for k in others):
        baseline = float(np.linalg.norm(centres[k2] - centres[k1]))
        if mode == "monocular":
            median = _median_depth(m, k2)
            if median is None or baseline / median < options.min_baseline_depth:
                continue
        elif baseline < camera.baseline:
            continue
        pairs = _match_for_triangulation(m, camera, k1, k2, options)
        if len(pairs) == 0:
            continue
        xyz, keep = _triangulate(m, camera, k1, k2, pairs, options=options)
        if np.any(keep):
            m = maps.add_points(m, xyz[keep], pairs[keep])
    return m


def triangulate_new_points(
    m: Map, camera: Camera, mode: str, options: Options, *, backend: str = "python"
) -> Map:
    """Triangulate the newest keyframe's free keypoints against the window's keyframes.

    ORB-SLAM's ``CreateNewMapPoints``. For each neighbour (the window's other keyframes,
    newest first, at most ``triangulation_neighbours``, ``mono_triangulation_neighbours`` for
    monocular): skip it if the baseline is too short (shorter than the stereo baseline, or
    under ``min_baseline_depth`` of its median depth for monocular);
    match (:func:`match_for_triangulation`); for each match take the point from the rays
    (:func:`slamlab.foundations.triangulation.two_view`) if their parallax beats what depth
    gives, else from the keypoint with the better depth; keep it if it is in front of both,
    passes the chi-square test in both, and its distances agree with the keypoints' levels
    (``scale_ratio``).

    Parameters
    ----------
    m : Map
        The map, its newest keyframe the current one.
    camera : slamlab.edge.frames.Camera
        The camera.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The thresholds.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map with the new points.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the mode is unknown, or the map has no keyframe.
    """
    check_mode(mode)
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "triangulate_new_points",
        reference=lambda: _triangulate_new_points(m, camera, mode, options),
        arguments=_arguments(m, camera, options, mode=mode),
        decode=maps.decode,
    )


def _fuse_target(m: Map, camera: Camera, k: int, p: int, options: Options) -> int | None:
    """Return the keypoint row of keyframe `k` that point row `p` fuses into, if any (``Fuse``)."""
    pose = m.kf_pose[k]
    xyz = m.pt_xyz[p]
    local = maps.to_camera(pose, xyz[None])
    if not local[0, 2] > 0.0:
        return None
    pixel = camera.project(local)[0]
    offset = xyz - maps.camera_centre(pose)
    distance = float(np.linalg.norm(offset))
    if not (
        camera.inside(pixel[None])[0]
        and maps.in_distance_range(m, np.array([p]), np.array([distance]), options)[0]
        and maps.viewing_cos(m, np.array([p]), offset[None], np.array([distance]))[0]
        >= options.view_cos_limit
    ):
        return None
    level = int(maps.predict_level(m, np.array([p]), np.array([distance]))[0])
    rows = _keypoint_indices(m, k)
    near = matching.features_in_area(
        m.kp_xy[rows],
        m.kp_level[rows],
        pixel,
        options.fuse_window * m.scale_factor**level,
        min_level=level - 1,
        max_level=level,
    )
    candidates = rows[near]
    error = residuals.chi2(
        camera,
        np.repeat(local, len(candidates), axis=0),
        m.kp_xy[candidates],
        m.kp_right[candidates],
        residuals.sigma2(m, m.kp_level[candidates]),
    )
    candidates = candidates[error < residuals.thresholds(m.kp_right[candidates], options)]
    if len(candidates) == 0:
        return None
    hamming = matching.distances(m.pt_descriptor[p : p + 1], m.kp_descriptor[candidates])[0]
    best = int(np.argmin(hamming))
    return int(candidates[best]) if hamming[best] <= options.th_low else None


def _fuse_one(m: Map, camera: Camera, k: int, point_id: int, options: Options) -> Map:
    """Fuse one point into a keyframe: a free keypoint observes it, a held one merges two."""
    p = int(m.point_rows(np.array([point_id]))[0])
    if p < 0 or np.any(m.kp_point[_keypoint_indices(m, k)] == point_id):
        return m  # merged away, or already in the keyframe
    row = _fuse_target(m, camera, k, p, options)
    if row is None:
        return m
    held = int(m.kp_point[row])
    if held < 0:
        kp_point = m.kp_point.copy()
        kp_point[row] = point_id
        return replace(m, kp_point=kp_point)
    q = int(m.point_rows(np.array([held]))[0])
    weights = maps.observation_weight(m, np.array([q, p]))
    if weights[0] > weights[1]:
        return _merge_points(m, lose=point_id, keep=held)
    return _merge_points(m, lose=held, keep=point_id)


def _merge_points(m: Map, *, lose: int, keep: int) -> Map:
    """Replace one point by another (``MapPoint::Replace``).

    The lost point's observations pass to the kept one, except in keyframes that already see
    the kept one; its visible and found counts are added to the kept one's; it is erased, and
    the kept one refreshed.
    """
    keyframe_of = m.keypoint_keyframe()
    kp_point = m.kp_point.copy()
    kept_in = set(keyframe_of[kp_point == keep].tolist())
    for row in np.flatnonzero(kp_point == lose):
        kp_point[row] = -1 if int(keyframe_of[row]) in kept_in else keep
    p_lose, p_keep = (int(r) for r in m.point_rows(np.array([lose, keep])))
    visible, found = m.pt_visible.copy(), m.pt_found.copy()
    visible[p_keep] += visible[p_lose]
    found[p_keep] += found[p_lose]
    merged = replace(m, kp_point=kp_point, pt_visible=visible, pt_found=found)
    erase = np.zeros(merged.points, dtype=bool)
    erase[p_lose] = True
    merged = maps.erase_points(merged, erase)
    return maps.refresh_points(merged, merged.point_rows(np.array([keep])))


def _fuse_points(m: Map, camera: Camera, options: Options) -> Map:
    """Fuse the newest keyframe's points with the window's (reference)."""
    newest_id = int(m.kf_id[-1])
    window_ids = m.kf_id[m.window(options.window)[:-1]]
    own = m.pt_id[m.observed_by(np.array([m.keyframes - 1]))]
    for target in window_ids:
        k = int(m.keyframe_rows(np.array([target]))[0])
        for point_id in own:
            m = _fuse_one(m, camera, k, int(point_id), options)
    newest = m.keyframes - 1
    rows = m.keyframe_rows(window_ids)
    theirs = m.pt_id[m.observed_by(rows[rows >= 0])]
    for point_id in theirs:
        m = _fuse_one(m, camera, newest, int(point_id), options)
    assert int(m.kf_id[-1]) == newest_id
    return maps.refresh_points(m, m.observed_by(np.array([newest])))


def fuse_points(m: Map, camera: Camera, options: Options, *, backend: str = "python") -> Map:
    """Fuse the newest keyframe's points with the window's (``SearchInNeighbors``, lite).

    The newest keyframe's points are fused into each other window keyframe, then all the
    others' points into the newest (``Fuse``): each point not already in the keyframe, in
    front, inside the image, in its distance range (:func:`slamlab.edge.map.in_distance_range`) and
    seen within ``view_cos_limit`` of its viewing direction, is compared with the keyframe's
    keypoints within ``fuse_window`` pixels at its predicted level (on that level or the one
    below) that pass the chi-square test; the nearest within ``th_low`` takes it: a free
    keypoint observes it, one observing another point merges the two (the one with the lower
    observation weight replaced). The newest keyframe's points are refreshed at the end.

    Parameters
    ----------
    m : Map
        The map.
    camera : slamlab.edge.frames.Camera
        The camera.
    options : Options
        The window and thresholds.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map after fusing.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the map has no keyframe.
    """
    _check_newest(m)
    return backends.call(
        backend,
        __name__,
        "fuse_points",
        reference=lambda: _fuse_points(m, camera, options),
        arguments=_arguments(m, camera, options),
        decode=maps.decode,
    )


def _redundant(m: Map, k: int, rows: IntArray, threshold: int) -> int:
    """Count a keyframe's points seen by enough other keyframes at its level or finer."""
    keyframe_of = m.keypoint_keyframe()
    point_rows = m.point_rows(m.kp_point[rows])
    weights = maps.observation_weight(m, point_rows)
    redundant = 0
    for row, p, weight in zip(rows, point_rows, weights, strict=True):
        if weight <= threshold:
            continue
        others = np.flatnonzero((m.kp_point == m.pt_id[p]) & (keyframe_of != k))
        if np.count_nonzero(m.kp_level[others] <= m.kp_level[row] + 1) >= threshold:
            redundant += 1
    return redundant


def _cull_keyframes(m: Map, camera: Camera, mode: str, options: Options) -> Map:
    """Cull the window's redundant keyframes (reference)."""
    limit = close_depth(camera, options)
    for kf_id in m.kf_id[m.window(options.window)][:-1]:
        if kf_id == 0:
            continue
        k = int(m.keyframe_rows(np.array([kf_id]))[0])
        rows = _keypoint_indices(m, k)
        rows = rows[m.kp_point[rows] >= 0]
        if mode != "monocular":
            depth = m.kp_depth[rows]
            rows = rows[(depth >= 0.0) & (depth <= limit)]  # NaN: no depth, not counted
        if len(rows) and _redundant(m, k, rows, options.redundant_observers) > (
            options.redundant_fraction * len(rows)
        ):
            m = maps.erase_keyframe(m, k)
    return m


def cull_keyframes(
    m: Map, camera: Camera, mode: str, options: Options, *, backend: str = "python"
) -> Map:
    """Cull the window's redundant keyframes (``KeyFrameCulling``).

    A window keyframe other than the newest and the map's first (id 0) goes when more than
    ``redundant_fraction`` of its points (its close ones with depth, for stereo and RGB-D)
    are observed with weight above ``redundant_observers`` and seen by at least
    ``redundant_observers`` other keyframes at its level, a finer one, or the next coarser.

    Parameters
    ----------
    m : Map
        The map.
    camera : slamlab.edge.frames.Camera
        The camera.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The fraction and count.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map without the redundant keyframes.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the mode is unknown.
    """
    check_mode(mode)
    return backends.call(
        backend,
        __name__,
        "cull_keyframes",
        reference=lambda: _cull_keyframes(m, camera, mode, options),
        arguments=_arguments(m, camera, options, mode=mode),
        decode=maps.decode,
    )


def _process(  # noqa: PLR0913 -- the stage's state
    m: Map, camera: Camera, frame: Frame, tracked: Tracked, *, mode: str, options: Options
) -> Map:
    """Insert a keyframe and run local mapping on it (reference)."""
    m = maps.insert_keyframe(m, frame, tracked.pose, tracked.points)
    if mode != "monocular":
        m = _create_points_from_depth(m, camera, options)
    m = _cull_points(m, mode, options)
    m = _triangulate_new_points(m, camera, mode, options)
    m = _fuse_points(m, camera, options)
    if m.keyframes >= options.ba_min_keyframes:
        m = adjust.bundle_adjust(m, camera, m.window(options.window), m.kf_id == 0, options)
    return _cull_keyframes(m, camera, mode, options)


def process(  # noqa: PLR0913 -- the stage's state, and the backend
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    backend: str = "python",
) -> Map:
    """Insert a keyframe and run local mapping on it (the module docstring's six steps).

    Parameters
    ----------
    m : Map
        The map.
    camera : slamlab.edge.frames.Camera
        The camera.
    frame : slamlab.edge.frames.Frame
        The new keyframe's frame.
    tracked : slamlab.edge.frames.Tracked
        Its pose and matches.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The settings.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Map
        The map after local mapping.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the mode is unknown, or the matches are not one per keypoint.
    """
    check_mode(mode)
    checked = checked_tracked(tracked, frame)
    return backends.call(
        backend,
        __name__,
        "process",
        reference=lambda: _process(m, camera, frame, checked, mode=mode, options=options),
        arguments=_arguments(
            m, camera, options, frame=frame.as_dict(), tracked=checked.as_dict(), mode=mode
        ),
        decode=maps.decode,
    )
