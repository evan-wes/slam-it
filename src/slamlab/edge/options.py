"""The edge pipeline's settings: every threshold named, defaulted to ORB-SLAM's.

ORB-SLAM2 (Mur-Artal and Tardós, T-RO 2017) is the reference: each setting's docstring names
the ORB-SLAM quantity it stands for. Where slamlab's lite pipeline has no counterpart (the
sliding window), the default is the design note's (``docs/design/2026-10-05-edge-vo.md``).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields
from typing import Any

from slamlab.foundations import orb
from slamlab.foundations.arrays import as_count, as_number
from slamlab.foundations.errors import InvalidInputError

MODES = ("monocular", "stereo", "rgbd")
"""The pipeline's modes."""

TH_LOW = 50
"""ORB-SLAM's stricter descriptor distance (initialisation, triangulation, fusion)."""

TH_HIGH = 100
"""Its looser one (tracking)."""


@dataclass(frozen=True)
class Options:
    """The edge pipeline's settings.

    Attributes
    ----------
    n_features : int
        ORB features per frame (ORB-SLAM's ``nFeatures``; 1000 for 640x480).
    levels : int
        Pyramid levels (``nLevels``).
    scale_factor : float
        The pyramid's scale factor (``scaleFactor``).
    fast_threshold, min_fast_threshold : int
        FAST thresholds (``iniThFAST``, ``minThFAST``).
    close_depth : float
        Keypoints nearer than this many baselines are "close" (``ThDepth``, 40): only close
        points become map points from depth, and their tracking decides keyframes.
    init_feature_factor : int
        A monocular frame is extracted with this many times ``n_features`` until the map
        exists (ORB-SLAM's initialisation extractor, ``2 * nFeatures``).
    min_init_features : int
        Keypoints a monocular frame needs to take part in initialisation (ORB-SLAM: 100).
    min_init_features_depth : int
        Keypoints a stereo or RGB-D frame needs to start the map, a point made at each one
        with depth (``StereoInitialization``'s 500 counts every keypoint).
    init_window : float
        The window, in pixels, in which a monocular initialisation matches a keypoint of the
        reference frame (``SearchForInitialization``'s 100).
    init_ratio : float
        Its ratio test (``ORBmatcher(0.9, true)``).
    init_min_matches : int
        Matches a monocular initialisation needs (100). With fewer, the reference frame is
        replaced by the current one; otherwise it is kept, and later frames are tried
        against it until the two initialise (ORB-SLAM's ``MonocularInitialization``); the
        map then needs as many points after its first adjustment.
    init_parallax_cos : float
        A monocular start keeps every inlier in front of both views whose two rays' cosine is
        below this (``CheckRT``'s 0.99998, 0.36 degrees); two-view's own 1 degree only decides
        whether the views initialise.
    motion_window : float
        The search window, in pixels at level 0, around a point predicted by the motion model
        (``TrackWithMotionModel``'s ``th``: 15, or 7 for stereo).
    stereo_motion_window : float
        The same, for stereo.
    min_motion_matches : int
        Matches the motion model needs before refining the pose (20; tried again with twice
        the window first).
    min_motion_inliers : int
        Inliers tracking the last frame must keep after refinement (``TrackWithMotionModel``'s
        10).
    local_window : float
        The search window when tracking the local map (``SearchLocalPoints``' ``th``: 1, or 3
        for RGB-D), scaled by the predicted level and the viewing angle.
    rgbd_local_window : float
        The same, for RGB-D.
    recovery_local_window : float
        The same on a recovery's frame and the next, in every mode (``SearchLocalPoints``'
        ``th = 5`` just after a relocalisation).
    recovery_window_frames : int
        The frames, the recovery's included, searched in ``recovery_local_window``
        (``SearchLocalPoints``' ``mnLastRelocFrameId + 2``).
    aligned_view_cos : float
        A point seen within this cosine of its mean viewing direction is searched in
        ``aligned_view_radius`` times the window, the others in ``oblique_view_radius`` times
        it (``RadiusByViewingCos``).
    aligned_view_radius : float
        See ``aligned_view_cos`` (2.5).
    oblique_view_radius : float
        See ``aligned_view_cos`` (4).
    local_map_ratio : float
        The local map search's ratio test, when the second best is on the same level
        (``TrackLocalMap``'s ``ORBmatcher(0.8)``).
    min_tracked : int
        Inliers the local map must give for the frame to be tracked (``TrackLocalMap``'s 30).
    recovery_tracked : int
        Inliers a frame needs within ``max_frames`` of a recovery (``TrackLocalMap``'s 50).
    chi2_mono : float
        The outlier test for a 2D observation (``5.991``: chi-square, 2 dof, 95%).
    chi2_stereo : float
        The same for a 3D (left and right) one (``7.815``: 3 dof, 95%).
    pose_rounds : int
        Rounds of pose refinement and outlier classification (``PoseOptimization``'s 4).
    view_cos_limit : float
        A map point is in view only when the angle to its mean viewing direction has at least
        this cosine (``isInFrustum``'s 0.5).
    min_distance_margin : float
        A map point is in view (and fusable) only from at least this fraction of its minimum
        distance (``isInFrustum``'s 0.8)...
    max_distance_margin : float
        ...to at most this multiple of its maximum (1.2).
    max_frames : int
        Frames after a recovery during which a frame needs ``recovery_tracked`` inliers, and
        no keyframe is made once the map holds more than this many (``mMaxFrames``, the
        frame rate, in ``TrackLocalMap`` and ``NeedNewKeyFrame``). Its other use in
        ``NeedNewKeyFrame``, condition c1a, only allows a keyframe; with local mapping idle,
        as the one-thread pipeline always is, c1b allows one anyway, and c2 decides.
    ref_ratio : float
        A keyframe is needed when the frame tracks fewer than this fraction of the reference
        keyframe's points (``thRefRatio``: 0.75 for stereo and RGB-D)...
    mono_ref_ratio : float
        ...0.9 for monocular.
    first_ref_ratio : float
        ...0.4 while the map has a single keyframe (``NeedNewKeyFrame``).
    ref_min_observations : int
        The reference keyframe's points counted are those observed at least this many times
        (``nMinObs``: 3)...
    ref_min_observations_early : int
        ...2 while the map holds two keyframes or fewer.
    min_keyframe_inliers : int
        ...and still tracks more than this many (15).
    close_tracked, close_untracked : int
        Stereo and RGB-D also need one when fewer than ``close_tracked`` close points are
        tracked and more than ``close_untracked`` could be created (100, 70).
    window : int
        The sliding window's keyframes: the local map, local BA and culling act on the last
        ``window`` keyframes (lite: ORB-SLAM's covisibility graph).
    min_new_points : int
        A stereo or RGB-D keyframe creates points from depth, nearest first, until it has at
        least this many and has passed the close depth (100).
    triangulation_neighbours : int
        Keyframes a new stereo or RGB-D keyframe triangulates against, newest first
        (``CreateNewMapPoints``' ``nn``: 10)...
    mono_triangulation_neighbours : int
        ...20 for monocular.
    epipolar_chi2 : float
        A match for triangulation must lie within this chi-square (1 dof, 95%) of its
        epipolar line (``CheckDistEpipolarLine``'s 3.84)...
    min_epipole_distance2 : float
        ...and, without depth, at least this squared distance from the epipole, in pixels at
        its level (``SearchForTriangulation``'s 100).
    min_baseline_depth : float
        A monocular keyframe triangulates only against one whose baseline is at least this
        fraction of that keyframe's median depth (``CreateNewMapPoints``' 0.01).
    max_parallax_cos : float
        Rays whose cosine is above this are too parallel to triangulate (0.9998).
    scale_ratio : float
        A triangulated point is kept only if its distances to the two keyframes agree with
        their keypoints' levels within this factor (``ratioFactor``: 1.5 times the scale
        factor).
    fuse_window : float
        The search window, in pixels at level 0, when fusing points into a keyframe (3).
    ba_iterations : int
        Local BA's iterations before dropping outliers (5)...
    ba_refine_iterations : int
        ...and after (10).
    ba_min_keyframes : int
        Local BA runs once the map holds this many keyframes (``LocalMapping``'s
        ``KeyFramesInMap() > 2``).
    init_ba_iterations : int
        The monocular map's first bundle adjustment's iterations (``GlobalBundleAdjustemnt(map,
        20)``).
    found_ratio : float
        A new point is culled when found in less than this fraction of the frames that should
        have seen it (0.25).
    recent_point_keyframes : int
        A point is culled on evidence while created at most this many keyframes ago
        (``MapPointCulling``' recent list: 3).
    min_point_observations : int
        A point two keyframes old is culled with this few observations or fewer (``cnThObs``:
        2 for monocular)...
    min_point_observations_depth : int
        ...3 for stereo and RGB-D.
    redundant_fraction : float
        A keyframe is culled when this fraction of its points are seen by
        ``redundant_observers`` other keyframes at the same or a finer scale (0.9, 3).
    redundant_observers : int
        See ``redundant_fraction``.
    pnp_threshold : float
        The PnP fallback's RANSAC threshold, in pixels.
    pnp_min_inliers : int
        Inliers the PnP fallback needs.
    relocalize_ratio : float
        The fallback's descriptor ratio test (``Relocalization``'s ``ORBmatcher(0.75, true)``).
    lost_frames : int
        Frames the fallback may fail on in a row before the track is lost and the run ends.
    seed : int
        Every RANSAC's seed, so a run is deterministic.
    """

    n_features: int = 1000
    levels: int = 8
    scale_factor: float = 1.2
    fast_threshold: int = 20
    min_fast_threshold: int = 7
    close_depth: float = 40.0
    init_feature_factor: int = 2
    min_init_features: int = 100
    min_init_features_depth: int = 500
    init_window: float = 100.0
    init_ratio: float = 0.9
    init_min_matches: int = 100
    init_parallax_cos: float = 0.99998
    motion_window: float = 15.0
    stereo_motion_window: float = 7.0
    min_motion_matches: int = 20
    min_motion_inliers: int = 10
    local_window: float = 1.0
    rgbd_local_window: float = 3.0
    recovery_local_window: float = 5.0
    recovery_window_frames: int = 2
    aligned_view_cos: float = 0.998
    aligned_view_radius: float = 2.5
    oblique_view_radius: float = 4.0
    local_map_ratio: float = 0.8
    min_tracked: int = 30
    recovery_tracked: int = 50
    chi2_mono: float = 5.991
    chi2_stereo: float = 7.815
    pose_rounds: int = 4
    view_cos_limit: float = 0.5
    min_distance_margin: float = 0.8
    max_distance_margin: float = 1.2
    max_frames: int = 30
    ref_ratio: float = 0.75
    mono_ref_ratio: float = 0.9
    first_ref_ratio: float = 0.4
    ref_min_observations: int = 3
    ref_min_observations_early: int = 2
    min_keyframe_inliers: int = 15
    close_tracked: int = 100
    close_untracked: int = 70
    window: int = 10
    min_new_points: int = 100
    triangulation_neighbours: int = 10
    mono_triangulation_neighbours: int = 20
    epipolar_chi2: float = 3.84
    min_epipole_distance2: float = 100.0
    min_baseline_depth: float = 0.01
    max_parallax_cos: float = 0.9998
    scale_ratio: float = 1.5
    fuse_window: float = 3.0
    ba_iterations: int = 5
    ba_refine_iterations: int = 10
    ba_min_keyframes: int = 3
    init_ba_iterations: int = 20
    found_ratio: float = 0.25
    recent_point_keyframes: int = 3
    min_point_observations: int = 2
    min_point_observations_depth: int = 3
    redundant_fraction: float = 0.9
    redundant_observers: int = 3
    pnp_threshold: float = 4.0
    pnp_min_inliers: int = 30
    relocalize_ratio: float = 0.75
    lost_frames: int = 5
    seed: int = 0

    def __post_init__(self) -> None:
        """Check every setting's type and range.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting has the wrong type or is out of range.
        """
        orb.Options(
            self.n_features,
            self.levels,
            self.scale_factor,
            self.fast_threshold,
            self.min_fast_threshold,
        )
        set_ = object.__setattr__  # frozen: store the checked, normalised values
        for item in fields(self):
            value = getattr(self, item.name)
            if item.type == "int":
                low = 0 if item.name in _MAY_BE_ZERO else 1
                set_(self, item.name, as_count(value, item.name, low, 1 << 30))
            else:
                number = as_number(value, item.name)
                if item.name in _FRACTIONS:
                    if not (math.isfinite(number) and 0.0 < number <= 1.0):
                        raise InvalidInputError(f"{item.name}: must be in (0, 1], got {number}")
                elif not (math.isfinite(number) and number > 0.0):
                    raise InvalidInputError(
                        f"{item.name}: must be finite and positive, got {number}"
                    )
                set_(self, item.name, number)

    def extraction(self, *, initializing: bool = False) -> orb.Options:
        """Return the ORB extraction settings.

        Parameters
        ----------
        initializing : bool, optional
            For a monocular frame before the map exists: ``init_feature_factor`` times the
            features.

        Returns
        -------
        slamlab.foundations.orb.Options
            Features, levels, scale factor and FAST thresholds.
        """
        factor = self.init_feature_factor if initializing else 1
        return orb.Options(
            min(self.n_features * factor, orb.MAX_FEATURES),
            self.levels,
            self.scale_factor,
            self.fast_threshold,
            self.min_fast_threshold,
        )

    def as_dict(self) -> dict[str, Any]:
        """Return the settings as a dict of scalars, as they cross the bindings.

        Returns
        -------
        dict
            Each field's value.
        """
        return asdict(self)


_MAY_BE_ZERO = frozenset({"seed", "min_init_features", "min_init_features_depth"})
"""The integer settings that may be 0."""

_FRACTIONS = frozenset(
    {
        "init_ratio",
        "relocalize_ratio",
        "min_baseline_depth",
        "first_ref_ratio",
        "min_distance_margin",
        "local_map_ratio",
        "aligned_view_cos",
        "init_parallax_cos",
        "ref_ratio",
        "mono_ref_ratio",
        "found_ratio",
        "redundant_fraction",
        "view_cos_limit",
        "max_parallax_cos",
    }
)
"""The real settings in (0, 1]."""
