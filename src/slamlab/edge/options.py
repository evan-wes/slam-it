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
    min_depth_features : int
        Keypoints with depth the first stereo or RGB-D keyframe needs (ORB-SLAM: 500).
    init_window : float
        The window, in pixels, in which a monocular initialisation matches a keypoint of the
        reference frame (``SearchForInitialization``'s 100).
    init_ratio : float
        Its ratio test (``ORBmatcher(0.9, true)``).
    init_min_matches : int
        Matches a monocular initialisation needs (100). With fewer, the reference frame is
        replaced by the current one; otherwise it is kept, and later frames are tried
        against it until the two initialise (ORB-SLAM's ``MonocularInitialization``).
    motion_window : float
        The search window, in pixels at level 0, around a point predicted by the motion model
        (``TrackWithMotionModel``'s ``th``: 15, or 7 for stereo).
    stereo_motion_window : float
        The same, for stereo.
    min_motion_matches : int
        Matches the motion model needs before refining the pose (20; tried again with twice
        the window first).
    local_window : float
        The search window when tracking the local map (``SearchLocalPoints``' ``th``: 1, or 3
        for RGB-D), scaled by the predicted level and the viewing angle.
    rgbd_local_window : float
        The same, for RGB-D.
    min_tracked : int
        Inliers the local map must give for the frame to be tracked (``TrackLocalMap``'s 30).
    chi2_mono : float
        The outlier test for a 2D observation (``5.991``: chi-square, 2 dof, 95%).
    chi2_stereo : float
        The same for a 3D (left and right) one (``7.815``: 3 dof, 95%).
    pose_rounds : int
        Rounds of pose refinement and outlier classification (``PoseOptimization``'s 4).
    view_cos_limit : float
        A map point is in view only when the angle to its mean viewing direction has at least
        this cosine (``isInFrustum``'s 0.5).
    max_frames : int
        A keyframe is due at the latest this many frames after the last (``mMaxFrames``: the
        frame rate).
    ref_ratio : float
        A keyframe is needed when the frame tracks fewer than this fraction of the reference
        keyframe's points (``thRefRatio``: 0.75 for stereo and RGB-D)...
    mono_ref_ratio : float
        ...0.9 for monocular.
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
    found_ratio : float
        A new point is culled when found in less than this fraction of the frames that should
        have seen it (0.25).
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
    min_depth_features: int = 500
    init_window: float = 100.0
    init_ratio: float = 0.9
    init_min_matches: int = 100
    motion_window: float = 15.0
    stereo_motion_window: float = 7.0
    min_motion_matches: int = 20
    local_window: float = 1.0
    rgbd_local_window: float = 3.0
    min_tracked: int = 30
    chi2_mono: float = 5.991
    chi2_stereo: float = 7.815
    pose_rounds: int = 4
    view_cos_limit: float = 0.5
    max_frames: int = 30
    ref_ratio: float = 0.75
    mono_ref_ratio: float = 0.9
    min_keyframe_inliers: int = 15
    close_tracked: int = 100
    close_untracked: int = 70
    window: int = 10
    min_new_points: int = 100
    max_parallax_cos: float = 0.9998
    scale_ratio: float = 1.5
    fuse_window: float = 3.0
    ba_iterations: int = 5
    ba_refine_iterations: int = 10
    found_ratio: float = 0.25
    min_point_observations: int = 2
    min_point_observations_depth: int = 3
    redundant_fraction: float = 0.9
    redundant_observers: int = 3
    pnp_threshold: float = 4.0
    pnp_min_inliers: int = 30
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


_MAY_BE_ZERO = frozenset({"seed", "min_init_features", "min_depth_features"})
"""The integer settings that may be 0."""

_FRACTIONS = frozenset(
    {
        "init_ratio",
        "ref_ratio",
        "mono_ref_ratio",
        "found_ratio",
        "redundant_fraction",
        "view_cos_limit",
        "max_parallax_cos",
    }
)
"""The real settings in (0, 1]."""
