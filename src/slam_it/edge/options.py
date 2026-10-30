"""The edge pipeline's settings: every threshold named, defaulted to ORB-SLAM's.

ORB-SLAM2 (Mur-Artal and Tardós, T-RO 2017) is the reference: each setting's docstring names
the ORB-SLAM quantity it stands for. Where ORB-SLAM has none, the default is the one decided in
the design that added the setting (``docs/design/``).

The settings are grouped by the stage that reads them (:class:`Options`), and a setting is
named by its group and its field, ``relocalization.min_inliers``: on a command line
(``--option``), in an error, and as the three languages' tables list them. A group is checked
when the :class:`Options` holding it is made.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field, fields, is_dataclass, replace
from typing import Any, get_type_hints

from slam_it.foundations import matching, orb
from slam_it.foundations import stereo as rows
from slam_it.foundations.arrays import MAX_IMAGE_SIDE, as_count, as_flag, as_number
from slam_it.foundations.errors import InvalidInputError

MODES = ("monocular", "stereo", "rgbd")
"""The pipeline's modes."""

FEATURES_AREA = 640 * 480
"""The frame area the default ``features.n_features``, ORB-SLAM's 1,000, is set for: its TUM
settings'."""

FEATURES_BOUNDS = (1000, 2000)
"""The fewest and most features :func:`features_for` gives. ORB-SLAM's own settings span 1,000
(TUM, 640x480) to 2,000 (KITTI, 1241x376). The fewest also keep a stereo or RGB-D frame able
to start the map, which needs more than ``initialization.min_features_depth`` (500)
keypoints."""


@dataclass(frozen=True)
class FeatureOptions:
    """ORB extraction's settings (``Options.features``).

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
    init_factor : int
        A monocular frame is extracted with this many times ``features.n_features`` until the map
        exists (ORB-SLAM's initialisation extractor, ``2 * nFeatures``).
    """

    n_features: int = 1000
    levels: int = 8
    scale_factor: float = 1.2
    fast_threshold: int = 20
    min_fast_threshold: int = 7
    init_factor: int = 2


@dataclass(frozen=True)
class MatchingOptions:
    """The descriptor distances matching takes in every stage (``Options.matching``).

    Attributes
    ----------
    th_low : int
        The stricter descriptor distance, for initialisation, triangulation and fusion
        (ORB-SLAM's ``ORBmatcher::TH_LOW``, 50), at most
        :data:`slam_it.foundations.matching.MAX_DISTANCE`.
    th_high : int
        The looser one, for tracking (``TH_HIGH``, 100).
    """

    th_low: int = 50
    th_high: int = 100


@dataclass(frozen=True)
class StereoOptions:
    """A stereo pair's row matching, and the close depth of stereo and RGB-D (``Options.stereo``).

    Attributes
    ----------
    max_distance, band, window, search, outlier_factor
        The stereo pair's row matching (:class:`slam_it.foundations.stereo.Options`, ORB-SLAM's
        ``ComputeStereoMatches``: a Hamming distance below 75, a band of 2 times the level's
        scale, a SAD window of 5 searched over 5 pixels either side, matches beyond 2.1 times
        the median SAD dropped).
    close_depth : float
        Keypoints nearer than this many baselines are "close" (``ThDepth``, 40): only close
        points become map points from depth, and their tracking decides keyframes.
    """

    max_distance: int = 75
    band: float = 2.0
    window: int = 5
    search: int = 5
    outlier_factor: float = 2.1
    close_depth: float = 40.0


@dataclass(frozen=True)
class InitializationOptions:
    """Starting the map (``Options.initialization``).

    Attributes
    ----------
    min_features : int
        A monocular frame takes part in initialisation with more than this many keypoints
        (``MonocularInitialization``'s ``mvKeys.size() > 100``).
    min_features_depth : int
        A stereo or RGB-D frame starts the map with more than this many keypoints, a point
        made at each one with depth (``StereoInitialization``'s ``N > 500`` counts every
        keypoint, with depth or not). A map started from too little depth to track loses the
        track and is reset (``loss.reset_keyframes``), as ORB-SLAM's is.
    window : float
        The window, in pixels, in which a monocular initialisation matches a keypoint of the
        reference frame (``SearchForInitialization``'s 100).
    ratio : float
        Its ratio test (``ORBmatcher(0.9, true)``).
    min_matches : int
        Matches a monocular initialisation needs (100). With fewer, the reference frame is
        replaced by the current one; otherwise it is kept, and later frames are tried
        against it until the two initialise (ORB-SLAM's ``MonocularInitialization``); the
        map then needs as many points after its first adjustment.
    parallax_cos : float
        A monocular start keeps every inlier in front of both views whose two rays' cosine is
        below this (``CheckRT``'s 0.99998, 0.36 degrees); two-view's own 1 degree only decides
        whether the views initialise.
    ba_iterations : int
        The monocular map's first bundle adjustment's iterations (``GlobalBundleAdjustemnt(map,
        20)``).
    ransac_iterations : int
        The most samples a monocular start's two-view RANSAC draws, per model (ORB-SLAM's
        ``TwoViewReconstruction``, 200): the start stops sooner once confident, and a pair that
        is not one costs at most this many.
    """

    min_features: int = 100
    min_features_depth: int = 500
    window: float = 100.0
    ratio: float = 0.9
    min_matches: int = 100
    parallax_cos: float = 0.99998
    ba_iterations: int = 20
    ransac_iterations: int = 200


@dataclass(frozen=True)
class TrackingOptions:
    """Placing each frame: the motion model, the local map, the PnP fallback (``Options.tracking``).

    Attributes
    ----------
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
        The frames, the recovery's included, searched in ``tracking.recovery_local_window``
        (``SearchLocalPoints``' ``mnLastRelocFrameId + 2``).
    aligned_view_cos : float
        A point seen at a cosine above this to its mean viewing direction is searched in
        ``tracking.aligned_view_radius`` times the window, the others in
        ``tracking.oblique_view_radius`` times it (``RadiusByViewingCos``).
    aligned_view_radius : float
        See ``tracking.aligned_view_cos`` (2.5).
    oblique_view_radius : float
        See ``tracking.aligned_view_cos`` (4).
    local_map_ratio : float
        The local map search's ratio test, when the second best is on the same level
        (``TrackLocalMap``'s ``ORBmatcher(0.8)``).
    min_tracked : int
        Inliers the local map must give for the frame to be tracked (``TrackLocalMap``'s 30).
    recovery_tracked : int
        Inliers a frame needs within ``tracking.max_frames`` of a recovery (``TrackLocalMap``'s 50).
    view_cos_limit : float
        A map point is in view only when the angle to its mean viewing direction has at least
        this cosine (``isInFrustum``'s 0.5).
    min_distance_margin : float
        A map point is in view (and fusable) only from at least this fraction of its minimum
        distance (``isInFrustum``'s 0.8)...
    max_distance_margin : float
        ...to at most this multiple of its maximum (1.2).
    max_frames : int
        Frames after a recovery during which a frame needs ``tracking.recovery_tracked``
        inliers, and no keyframe is made once the map holds more than this many (``mMaxFrames``,
        the frame rate, in ``TrackLocalMap`` and ``NeedNewKeyFrame``). Its other use in
        ``NeedNewKeyFrame``, condition c1a, only allows a keyframe; with local mapping idle,
        as the one-thread pipeline always is, c1b allows one anyway, and c2 decides.
    pnp_threshold : float
        The PnP fallback's RANSAC threshold, in pixels.
    pnp_min_inliers : int
        Inliers the PnP fallback needs.
    """

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
    max_frames: int = 30
    view_cos_limit: float = 0.5
    min_distance_margin: float = 0.8
    max_distance_margin: float = 1.2
    pnp_threshold: float = 4.0
    pnp_min_inliers: int = 30


@dataclass(frozen=True)
class AdjustmentOptions:
    """Pose optimisation and local BA (``Options.adjustment``).

    Attributes
    ----------
    chi2_mono : float
        The outlier test for a 2D observation (``5.991``: chi-square, 2 dof, 95%).
    chi2_stereo : float
        The same for a 3D (left and right) one (``7.815``: 3 dof, 95%).
    pose_rounds : int
        Rounds of pose refinement and outlier classification (``PoseOptimization``'s 4).
    iterations : int
        Local BA's iterations before dropping outliers (5)...
    refine_iterations : int
        ...and after (10).
    min_keyframes : int
        Local BA runs once the map holds this many keyframes (``LocalMapping``'s
        ``KeyFramesInMap() > 2``).
    """

    chi2_mono: float = 5.991
    chi2_stereo: float = 7.815
    pose_rounds: int = 4
    iterations: int = 5
    refine_iterations: int = 10
    min_keyframes: int = 3


@dataclass(frozen=True)
class KeyframeOptions:
    """The keyframe decision (``Options.keyframes``).

    Attributes
    ----------
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
    min_inliers : int
        A frame becomes a keyframe only if it tracks more than this many inliers
        (``NeedNewKeyFrame``'s 15).
    close_tracked, close_untracked : int
        Stereo and RGB-D also need one when fewer than ``keyframes.close_tracked`` close points are
        tracked and more than ``keyframes.close_untracked`` could be created (100, 70).
    busy_ref_ratio : float
        With local mapping busy, in threads of their own, a stereo or RGB-D frame tracking fewer
        than this fraction of its reference keyframe's points still asks for one
        (``NeedNewKeyFrame``'s ``c1c``: 0.25).
    """

    ref_ratio: float = 0.75
    mono_ref_ratio: float = 0.9
    first_ref_ratio: float = 0.4
    ref_min_observations: int = 3
    ref_min_observations_early: int = 2
    min_inliers: int = 15
    close_tracked: int = 100
    close_untracked: int = 70
    busy_ref_ratio: float = 0.25


@dataclass(frozen=True)
class MappingOptions:
    """Local mapping: points from depth, triangulation, fusion and culling (``Options.mapping``).

    Attributes
    ----------
    min_new_points : int
        A stereo or RGB-D keyframe creates points from depth, nearest first, until it has at
        least this many and has passed the close depth (100).
    triangulation_neighbours : int
        A new stereo or RGB-D keyframe triangulates against its best this many links
        (``CreateNewMapPoints``' ``nn``: 10)...
    mono_triangulation_neighbours : int
        ...20 for monocular (ORB-SLAM2's; ORB-SLAM3 takes 30, a departure kept, not measured on
        the test set: the online-SLAM design's section 11, L2).
    epipolar_chi2 : float
        A match for triangulation must lie within this chi-square (1 dof, 95%) of its
        epipolar line (``CheckDistEpipolarLine``'s 3.84)...
    min_epipole_distance2 : float
        ...and, without depth, a squared distance from the epipole of at least this times
        the keypoint's level scale s^l (``SearchForTriangulation``'s ``100 * mvScaleFactors``,
        ORB-SLAM's mixed units).
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
    fuse_neighbours : int
        A new stereo or RGB-D keyframe's points are fused with its best this many links'
        (``SearchInNeighbors``' ``nn``: 10)...
    mono_fuse_neighbours : int
        ...20 for monocular...
    fuse_second_neighbours : int
        ...and with each one's best this many links' (5): ORB-SLAM2's numbers, as triangulation's
        are (ORB-SLAM3's are 30 for monocular, and 20).
    found_ratio : float
        A new point is culled when found in less than this fraction of the frames that should
        have seen it (0.25).
    recent_point_keyframes : int
        A point is culled on evidence while created at most this many keyframes ago
        (``MapPointCulling``' recent list: 3).
    point_cull_age : int
        A recent point at least this many keyframes old is culled on too few observations
        (``MapPointCulling``' 2).
    min_point_observations : int
        A point ``mapping.point_cull_age`` keyframes old is culled with this few observations
        or fewer (``cnThObs``: 2 for monocular)...
    min_point_observations_depth : int
        ...3 for stereo and RGB-D.
    redundant_fraction : float
        The fraction of a keyframe's points that must be redundant for culling to remove it,
        each seen by ``mapping.redundant_observers`` other keyframes at the same, a finer or the
        next coarser scale (``KeyFrameCulling``'s 0.9).
    redundant_observers : int
        How many other keyframes must see a point, at the same, a finer or the next coarser
        scale, for it to count as redundant (4): ORB-SLAM3's ``nObs > 3`` (``KeyFrameCulling``),
        where ORB-SLAM2 has ``>= 3``. The point must also weigh more than this (``Observations()
        > 3`` there), which so many other keyframes and its own already make it. Why ORB-SLAM3's
        and not ORB-SLAM2's: the online-SLAM design, section 11's L2.
    max_queued : int
        With local mapping busy, in threads of their own, a stereo or RGB-D keyframe is queued
        only while fewer than this many wait (``NeedNewKeyFrame``'s ``KeyframesInQueue() < 3``).
    """

    min_new_points: int = 100
    triangulation_neighbours: int = 10
    mono_triangulation_neighbours: int = 20
    epipolar_chi2: float = 3.84
    min_epipole_distance2: float = 100.0
    min_baseline_depth: float = 0.01
    max_parallax_cos: float = 0.9998
    scale_ratio: float = 1.5
    fuse_window: float = 3.0
    fuse_neighbours: int = 10
    mono_fuse_neighbours: int = 20
    fuse_second_neighbours: int = 5
    found_ratio: float = 0.25
    recent_point_keyframes: int = 3
    point_cull_age: int = 2
    min_point_observations: int = 2
    min_point_observations_depth: int = 3
    redundant_fraction: float = 0.9
    redundant_observers: int = 4
    max_queued: int = 3


@dataclass(frozen=True)
class CovisibilityOptions:
    """The covisibility graph (``Options.covisibility``).

    Attributes
    ----------
    min_points : int
        Two keyframes that observe at least this many of the same points are linked
        (``UpdateConnections``' 15); one that shares this many with none is linked to the one it
        shares most with.
    local_keyframes : int
        Tracking's local map stops adding K1's neighbours once it holds more than this many
        keyframes (``UpdateLocalKeyFrames``' 80).
    local_neighbours : int
        Of a K1 keyframe's best this many links, the first not yet in the local map joins it
        (``GetBestCovisibilityKeyFrames(10)``).
    strong_points : int
        Two keyframes sharing at least this many points are strongly linked, the essential
        graph's links (``GetCovisiblesByWeight(100)``).
    """

    min_points: int = 15
    local_keyframes: int = 80
    local_neighbours: int = 10
    strong_points: int = 100


@dataclass(frozen=True)
class RelocalizationOptions:
    """Relocalising a failed frame against the whole map (``Options.relocalization``).

    Attributes
    ----------
    enabled : bool
        Relocalise a failed frame against the whole map through its keyframe database
        (:meth:`Options.keeps_keyframe_database`; on, as ORB-SLAM always is). Off, a failed frame
        tries only the motion model and the local fallback, and a map is lost after
        ``loss.lost_frames`` failures in a row, as before milestone 2B.
    ratio : float
        The descriptor ratio test of the fallback and of relocalisation's matching by word
        (``Relocalization``'s ``ORBmatcher(0.75, true)``).
    common_words : float
        Relocalisation scores the keyframes that share more than this fraction of the most
        words any keyframe shares with the frame (``DetectRelocalizationCandidates``' 0.8).
    neighbours : int
        A scored keyframe's score is accumulated over its best this many covisible keyframes
        (``GetBestCovisibilityKeyFrames(10)``).
    group_score : float
        Groups whose accumulated score exceeds this fraction of the best give a candidate
        (0.75).
    min_matches : int
        A candidate needs this many matches by word (``SearchByBoW``; 15).
    inlier_ratio : float
        A candidate's PnP RANSAC needs this fraction of its matches as inliers, and draws the
        samples that find such a pose with ``relocalization.confidence`` (MLPnP's ``epsilon``, 0.5).
    min_inliers : int
        ...and at least this many, before and after the pose is optimised (10).
    confidence : float
        The confidence its samples are drawn for, in (0, 1) (0.99).
    ransac_iterations : int
        The most samples it draws (300).
    window : float
        The window, in pixels at the predicted level, in which a candidate's other points are
        searched by projection when too few inliers relocalise the frame (10, with
        ``matching.th_high`` as the distance).
    narrow_inliers : int
        With more inliers than this but too few still, the search is repeated in a narrower
        window (``nGood > 30``).
    narrow_window : float
        That window (3).
    narrow_distance : int
        Its descriptor distance (64), at most
        :data:`slam_it.foundations.matching.MAX_DISTANCE`.
    """

    enabled: bool = True
    ratio: float = 0.75
    common_words: float = 0.8
    neighbours: int = 10
    group_score: float = 0.75
    min_matches: int = 15
    inlier_ratio: float = 0.5
    min_inliers: int = 10
    confidence: float = 0.99
    ransac_iterations: int = 300
    window: float = 10.0
    narrow_inliers: int = 30
    narrow_window: float = 3.0
    narrow_distance: int = 64


@dataclass(frozen=True)
class LoopOptions:
    """Loop closure: detection, the loop's similarity, its correction (``Options.loops``).

    ORB-SLAM3's ``LoopClosing`` (``NewDetectCommonRegions`` and what it calls), with the values
    its source hard-codes. A distance named here is a Hamming distance; ORB-SLAM writes the
    projection searches' as ``TH_LOW`` (50) times a ratio.

    A merge's welding window (``MergeLocal``'s) is built in three steps: the merge's keyframe
    and its best ``weld_neighbours`` links; then, while it holds fewer than ``weld_neighbours``
    keyframes, each member's best ``weld_widen`` links taken in; that at most ``weld_rounds``
    times.

    Attributes
    ----------
    enabled : bool
        Close loops in the map being tracked, each keyframe searched once it is mapped (on, as
        ORB-SLAM always is). It needs the keyframe database: on, it keeps the database whether
        relocalisation is on or off (:meth:`Options.keeps_keyframe_database`).
    min_keyframes : int
        A map of fewer keyframes is not searched (``KeyFramesInMap() < 12``).
    candidates : int
        The best groups of keyframes the database gives (``DetectNBestCandidates``' 3).
    common_words : float
        The database scores the keyframes sharing more than this fraction of the most words any
        shares with the keyframe (0.8).
    neighbours : int
        A candidate's group: it and its best this many links, whose scores the database
        accumulates too; and the new keyframe's best this many links, which confirm a loop
        (``GetBestCovisibilityKeyFrames(10)``).
    ratio : float
        The ratio test of the matching by word between the new keyframe and a group
        (``ORBmatcher matcherBoW(0.9, true)``), at ``matching.th_low``.
    min_matches : int
        A group needs this many matches by word (``nBoWMatches``, 20).
    min_inliers : int
        The loop's Sim(3) RANSAC needs this many inliers (``nBoWInliers``, 15)...
    confidence : float
        ...and draws the samples that find them with this confidence, in (0, 1) (0.99)...
    ransac_iterations : int
        ...at most this many (``SetRansacParameters(0.99, 15, 300)``).
    chi2 : float
        A match is the RANSAC's inlier when it reprojects within this many variances in both
        keyframes (``Sim3Solver``'s 9.21: two degrees of freedom at 99%).
    coarse_window : float
        The group's points are searched through the RANSAC's similarity in this window, in
        pixels at the predicted level (``SearchByProjection(..., 8, 1.5)``)...
    coarse_distance : int
        ...within this distance (75)...
    coarse_matches : int
        ...and must give this many matches (``nProjMatches``, 50).
    huber_chi2 : float
        The refinement's Huber loss sits at this chi-square's root, and a match whose either
        error exceeds it is dropped (``OptimizeSim3``'s ``th2``, 10).
    refine_iterations : int
        The refinement's first round (5), and its second when it dropped no match...
    refine_more : int
        ...or this many when it did (10).
    min_kept : int
        The refinement fails when fewer matches than this survive its first round (10).
    min_refined : int
        The refined similarity needs this many inliers (``nSim3Inliers``, 20)...
    fine_window : float
        ...and the group's points, searched through it in this window (5)...
    fine_distance : int
        ...within this distance (50)...
    fine_matches : int
        ...must give this many matches (``nProjOptMatches``, 80).
    confirm_window : float
        A neighbour of the new keyframe, or a later keyframe, confirms the loop when the group's
        points, searched in this window through the similarity carried to it
        (``FindMatchesByProjection``'s 3)...
    confirm_distance : int
        ...within this distance (75)...
    confirm_matches : int
        ...give this many matches (30).
    confirmations : int
        Confirmations that close the loop (``mnLoopNumCoincidences >= 3``).
    misses : int
        Later keyframes in a row that fail to confirm a loop drop it (``mnLoopNumNotFound >=
        2``).
    temporal_refined : int
        A later keyframe confirms only if refining the similarity carried to it keeps more
        inliers than this (``DetectAndReffineSim3FromLastKF``'s 50)...
    temporal_matches : int
        ...and the search through the refined one gives this many matches (100).
    fuse_window : float
        The loop's group's points are fused into the corrected keyframes in this window, within
        ``matching.th_low`` (``SearchAndFuse``'s 4).
    essential_points : int
        The essential graph links keyframes sharing this many points (``minFeat``, 100).
    essential_iterations : int
        Its Levenberg-Marquardt iterations (20)...
    essential_damping : float
        ...from this damping (``setUserLambdaInit(1e-16)``).
    global_ba : bool
        Bundle-adjust the whole map after each loop (on, as visual-only ORB-SLAM3 always does;
        off measures what it adds).
    global_iterations : int
        Global BA's Levenberg-Marquardt iterations at most (``GlobalBundleAdjustemnt``'s 10).
    global_damping_scale : float
        Global BA's ``τ``, relative: its first damping is this times the largest diagonal entry
        of its normal equations, then follows Nielsen's schedule (g2o's damping, its ``τ`` 1e-5,
        which ORB-SLAM3's global BA leaves at its default;
        :attr:`slam_it.foundations.graphs.Options.damping_scale`). Unlike
        ``loops.essential_damping``, which is absolute.
    global_small_steps : int
        How many iterations in a row, each lowering global BA's error by less than
        ``loops.global_small_decrease`` of it, end the solve (ORB-SLAM3's g2o stop, 3); 0, never.
    global_small_decrease : float
        The fraction of global BA's error an iteration must lower it by not to count as a small
        step towards ``loops.global_small_steps`` (ORB-SLAM3's 1e-3).
    min_depth_ratio : float
        The fraction of every observation's median depth below which global and welding BA leave
        an observation out at the start, with those behind their camera
        (:func:`slam_it.foundations.graphs.bundle_adjust`'s ``near``, 1e-3; 0 leaves none out).
        In [0, 1). ORB-SLAM3 has no such filter; why it is here: the online-SLAM design, section
        11's L2.
    weld_neighbours : int
        The best links of the merge's keyframe a welding window starts with, and the keyframes
        it is widened to while it holds fewer (ORB-SLAM3's ``numTemporalKFs``, 25, which
        ``MergeLocal`` uses for visual maps too).
    weld_widen : int
        The best links of each member a welding window takes in each time it is widened
        (``numTemporalKFs / 2``, 12).
    weld_rounds : int
        The most times a welding window is widened (``nMaxTries``, 5); 0 widens it none.
    """

    enabled: bool = True
    min_keyframes: int = 12
    candidates: int = 3
    common_words: float = 0.8
    neighbours: int = 10
    ratio: float = 0.9
    min_matches: int = 20
    min_inliers: int = 15
    confidence: float = 0.99
    ransac_iterations: int = 300
    chi2: float = 9.21
    coarse_window: float = 8.0
    coarse_distance: int = 75
    coarse_matches: int = 50
    huber_chi2: float = 10.0
    refine_iterations: int = 5
    refine_more: int = 10
    min_kept: int = 10
    min_refined: int = 20
    fine_window: float = 5.0
    fine_distance: int = 50
    fine_matches: int = 80
    confirm_window: float = 3.0
    confirm_distance: int = 75
    confirm_matches: int = 30
    confirmations: int = 3
    misses: int = 2
    temporal_refined: int = 50
    temporal_matches: int = 100
    fuse_window: float = 4.0
    essential_points: int = 100
    essential_iterations: int = 20
    essential_damping: float = 1e-16
    global_ba: bool = True
    global_iterations: int = 10
    global_damping_scale: float = 1e-5
    global_small_steps: int = 3
    global_small_decrease: float = 1e-3
    min_depth_ratio: float = 1e-3
    weld_neighbours: int = 25
    weld_widen: int = 12
    weld_rounds: int = 5


@dataclass(frozen=True)
class LossOptions:
    """When a lost track's map is closed (``Options.loss``).

    Attributes
    ----------
    relocalize_seconds : float
        Relocalising, a recently lost track is lost once more than this has passed since its
        first failed frame with none tracked (ORB-SLAM3's ``RECENTLY_LOST``, 3 s): the map is
        kept, or reset (``loss.reset_keyframes``), and the next frame starts a new one (ORB-SLAM3's
        atlas). 0 has no window: the map is lost on its first failed frame, a new map at once.
    lost_frames : int
        A map is lost after this many failed frames in a row instead when it has
        ``loss.reset_keyframes`` keyframes or fewer (ORB-SLAM3 loses one at once), or whenever
        ``relocalization.enabled`` is off.
    reset_keyframes : int
        A map lost with this many keyframes or fewer is reset rather than kept: its keyframes
        and points removed, its frames' poses dropped (ORB-SLAM3's ``ResetActiveMap``, for a
        map of up to 10 keyframes). 0 keeps every map.
    session_keyframes : int
        A map a session ends with fewer keyframes than this is reset rather than kept, when a
        run of several sequences starts the next (ORB-SLAM3's ``ChangeDataset``, 12). 0 keeps
        every map.
    """

    relocalize_seconds: float = 3.0
    lost_frames: int = 5
    reset_keyframes: int = 10
    session_keyframes: int = 12


@dataclass(frozen=True)
class Options:
    """The edge pipeline's settings, grouped by the stage that reads them.

    A stage may read another group's setting: relocalisation takes the PnP fallback's threshold
    (``tracking.pnp_threshold``), and the fallback relocalisation's ratio test
    (``relocalization.ratio``).

    Attributes
    ----------
    features : FeatureOptions
        ORB extraction.
    matching : MatchingOptions
        The descriptor distances.
    stereo : StereoOptions
        A stereo pair's row matching, and the close depth.
    initialization : InitializationOptions
        Starting the map.
    tracking : TrackingOptions
        Placing each frame.
    adjustment : AdjustmentOptions
        Pose optimisation and local BA.
    keyframes : KeyframeOptions
        The keyframe decision.
    mapping : MappingOptions
        Local mapping.
    covisibility : CovisibilityOptions
        The covisibility graph.
    relocalization : RelocalizationOptions
        Relocalisation.
    loops : LoopOptions
        Loop closure.
    loss : LossOptions
        When a lost track's map is closed.
    seed : int
        Every RANSAC's seed, so a run is deterministic.
    """

    features: FeatureOptions = field(default_factory=FeatureOptions)
    matching: MatchingOptions = field(default_factory=MatchingOptions)
    stereo: StereoOptions = field(default_factory=StereoOptions)
    initialization: InitializationOptions = field(default_factory=InitializationOptions)
    tracking: TrackingOptions = field(default_factory=TrackingOptions)
    adjustment: AdjustmentOptions = field(default_factory=AdjustmentOptions)
    keyframes: KeyframeOptions = field(default_factory=KeyframeOptions)
    mapping: MappingOptions = field(default_factory=MappingOptions)
    covisibility: CovisibilityOptions = field(default_factory=CovisibilityOptions)
    relocalization: RelocalizationOptions = field(default_factory=RelocalizationOptions)
    loops: LoopOptions = field(default_factory=LoopOptions)
    loss: LossOptions = field(default_factory=LossOptions)
    seed: int = 0

    def __post_init__(self) -> None:
        """Check every setting's type and range, each named by its group and field.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If a group is not its class, or a setting has the wrong type or is out of range.
        """
        for name, kind in _GROUPS.items():
            given = getattr(self, name)
            if not isinstance(given, kind):
                got = type(given).__name__
                raise InvalidInputError(f"{name}: expected {kind.__name__}, got {got}")
        for owner, item, name in _fields(self):
            value = _checked(name, item.type, getattr(owner, item.name))
            object.__setattr__(owner, item.name, value)  # frozen: store it checked, normalised
        try:
            self.extraction()  # its fields checked first, so a wrong type is named, not a TypeError
        except InvalidInputError as error:
            raise InvalidInputError(f"features.{error}") from error
        for name, value in (
            ("matching.th_low", self.matching.th_low),
            ("matching.th_high", self.matching.th_high),
            ("relocalization.narrow_distance", self.relocalization.narrow_distance),
            ("loops.coarse_distance", self.loops.coarse_distance),
            ("loops.fine_distance", self.loops.fine_distance),
            ("loops.confirm_distance", self.loops.confirm_distance),
        ):
            as_count(value, name, 1, matching.MAX_DISTANCE)
        for name, confidence in (
            ("relocalization.confidence", self.relocalization.confidence),
            ("loops.confidence", self.loops.confidence),
        ):
            if not confidence < 1.0:
                raise InvalidInputError(f"{name}: must be in (0, 1), got {confidence}")
        try:
            self.stereo_matching()
        except InvalidInputError as error:
            raise InvalidInputError(f"stereo.{error}") from error

    def keeps_keyframe_database(self) -> bool:
        """Return whether a pipeline with these settings keeps the keyframe database.

        It does when relocalising or closing loops, either alone, as ORB-SLAM always keeps its
        ``KeyFrameDatabase`` (B3's review panel, H7). Every pipeline takes a vocabulary whatever
        this says, as mapping quantises each keyframe by it (since 0.3.18.0; the online-SLAM
        design's section 11, L2): the one statement of the rule (B3's panel, run 2, H23).

        Returns
        -------
        bool
            ``relocalization.enabled`` or ``loops.enabled``.
        """
        return self.relocalization.enabled or self.loops.enabled

    def settings(self) -> dict[str, bool | int | float]:
        """Return every setting by its name, in the groups' order.

        Returns
        -------
        dict
            Each setting's name (``group.field``, or ``seed``) to its value.
        """
        return {name: getattr(owner, item.name) for owner, item, name in _fields(self)}

    def with_settings(self, changes: Mapping[str, Any]) -> Options:
        """Return these settings with some changed, each named as :meth:`settings` names it.

        Parameters
        ----------
        changes : mapping
            Each setting's name to its new value.

        Returns
        -------
        Options
            The settings, checked.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            For an unknown name, or a value the setting refuses.
        """
        known = self.settings()
        top: dict[str, Any] = {}
        grouped: dict[str, dict[str, Any]] = {}
        for name, value in changes.items():
            if name not in known:
                raise InvalidInputError(f"options: no setting {name!r}")
            group, dot, item = name.partition(".")
            if dot:
                grouped.setdefault(group, {})[item] = value
            else:
                top[name] = value
        for group, items in grouped.items():
            top[group] = replace(getattr(self, group), **items)
        return replace(self, **top)

    def extraction(self, *, initializing: bool = False) -> orb.Options:
        """Return the ORB extraction settings.

        Parameters
        ----------
        initializing : bool, optional
            For a monocular frame before the map exists: ``features.init_factor`` times the
            features.

        Returns
        -------
        slam_it.foundations.orb.Options
            Features, levels, scale factor and FAST thresholds.
        """
        features = self.features
        factor = features.init_factor if initializing else 1
        return orb.Options(
            min(features.n_features * factor, orb.MAX_FEATURES),
            features.levels,
            features.scale_factor,
            features.fast_threshold,
            features.min_fast_threshold,
        )

    def stereo_matching(self) -> rows.Options:
        """Return the stereo pair's row-matching settings.

        Returns
        -------
        slam_it.foundations.stereo.Options
            The ``stereo`` group's, but the close depth.
        """
        s = self.stereo
        return rows.Options(s.max_distance, s.band, s.window, s.search, s.outlier_factor)

    def as_dict(self) -> dict[str, Any]:
        """Return the settings as a dict of groups, as they cross the bindings and runs record them.

        Returns
        -------
        dict
            Each group's name to a dict of its fields' values, and ``seed``.
        """
        return asdict(self)


_GROUPS: dict[str, type] = {
    name: kind
    for name, kind in get_type_hints(Options).items()
    if isinstance(kind, type) and is_dataclass(kind)
}
"""Each group's name in :class:`Options`, and its class: its fields that are groups, all but the
seed."""


def _fields(options: Options) -> list[tuple[Any, Any, str]]:
    """Every setting: the object holding it, its field and its name, in the groups' order."""
    out = [
        (getattr(options, group), item, f"{group}.{item.name}")
        for group in _GROUPS
        for item in fields(getattr(options, group))
    ]
    seed = next(item for item in fields(options) if item.name == "seed")
    return [*out, (options, seed, "seed")]


def _checked(name: str, kind: str, value: Any) -> bool | int | float:  # noqa: ANN401 -- as given
    """Return one setting's value checked for its type and range, and normalised."""
    if kind == "bool":
        return as_flag(value, name)
    if kind == "int":
        return as_count(value, name, 0 if name in _MAY_BE_ZERO else 1, 1 << 30)
    number = as_number(value, name)
    if name in _MAY_BE_ZERO:
        if not (math.isfinite(number) and number >= 0.0):
            raise InvalidInputError(f"{name}: must be finite and not negative, got {number}")
    elif name in _BELOW_ONE:
        if not (math.isfinite(number) and 0.0 <= number < 1.0):
            raise InvalidInputError(f"{name}: must be in [0, 1), got {number}")
    elif name in _FRACTIONS:
        if not (math.isfinite(number) and 0.0 < number <= 1.0):
            raise InvalidInputError(f"{name}: must be in (0, 1], got {number}")
    elif not (math.isfinite(number) and number > 0.0):
        raise InvalidInputError(f"{name}: must be finite and positive, got {number}")
    return number


_MAY_BE_ZERO = frozenset(
    {
        "seed",
        "initialization.min_features",
        "initialization.min_features_depth",
        "loss.reset_keyframes",
        "loss.session_keyframes",
        "loops.weld_rounds",
        "loops.global_small_steps",
        "loss.relocalize_seconds",
    }
)
"""The settings that may be 0: integers, and one real."""

_BELOW_ONE = frozenset({"loops.min_depth_ratio"})
"""The real settings in [0, 1)."""

_FRACTIONS = frozenset(
    {
        "initialization.ratio",
        "relocalization.ratio",
        "mapping.min_baseline_depth",
        "keyframes.first_ref_ratio",
        "keyframes.busy_ref_ratio",
        "tracking.min_distance_margin",
        "tracking.local_map_ratio",
        "tracking.aligned_view_cos",
        "initialization.parallax_cos",
        "keyframes.ref_ratio",
        "keyframes.mono_ref_ratio",
        "mapping.found_ratio",
        "mapping.redundant_fraction",
        "tracking.view_cos_limit",
        "mapping.max_parallax_cos",
        "relocalization.common_words",
        "relocalization.group_score",
        "relocalization.inlier_ratio",
        "relocalization.confidence",
        "loops.common_words",
        "loops.ratio",
        "loops.confidence",
        "loops.global_small_decrease",
    }
)
"""The real settings in (0, 1]."""


def features_for(width: int, height: int) -> int:
    """Return the ORB features for a frame's size, ORB-SLAM's 1,000 at 640x480 scaled with area.

    ORB-SLAM's settings ask for more features on a larger frame (1,200 for EuRoC's 752x480,
    2,000 for KITTI's 1241x376); this scales the default ``features.n_features``, its 1,000, by the
    frame's area over :data:`FEATURES_AREA`, rounded, within :data:`FEATURES_BOUNDS`. A run is
    given it where it is set up, as its ``features.n_features``
    (:func:`slam_it.orchestration.vo.options_for`); the default stays 1,000, so a dataset's
    results do not move. It is Python's alone, by design: the cores take the count as the
    setting they already have, and never compute it.

    Parameters
    ----------
    width, height : int
        The frame's size, in pixels.

    Returns
    -------
    int
        The features: 1,000 at 640x480 and below, 1,334 at 854x480, and 2,000 from twice
        640x480's area up (a 720p frame).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a side is not a positive integer.
    """
    area = as_count(width, "width", 1, MAX_IMAGE_SIDE) * as_count(
        height, "height", 1, MAX_IMAGE_SIDE
    )
    low, high = FEATURES_BOUNDS
    return min(max(round(FeatureOptions.n_features * area / FEATURES_AREA), low), high)


def check_mode(mode: str) -> None:
    """Refuse an unknown mode.

    Parameters
    ----------
    mode : str
        The mode.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not one of :data:`MODES`.
    """
    if mode not in MODES:
        raise InvalidInputError(f"mode: expected one of {MODES}, got {mode!r}")
