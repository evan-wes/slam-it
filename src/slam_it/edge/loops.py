"""Loop closing: find the place a keyframe revisits, and pull the map onto it (ORB-SLAM3's).

ORB-SLAM3's ``LoopClosing`` (Campos et al., T-RO 2021, section VII; ``NewDetectCommonRegions``
and ``CorrectLoop``), run on each keyframe after local mapping (:mod:`slam_it.edge.mapping`),
in the pipeline's one worker. The design is section 10 of
``docs/design/2026-10-14-online-slam.md``.

**Detection** (:func:`detect`), on a map of ``loops.min_keyframes`` keyframes or more:

1. **A loop already being confirmed** (:class:`Pending`) is carried to the new keyframe
   (``DetectAndReffineSim3FromLastKF``): the similarity the last confirming keyframe had,
   moved by its motion since; the matched keyframe's group's points searched through it
   (:func:`search_by_similarity`, ``loops.confirm_window`` and ``loops.confirm_distance``),
   ``loops.confirm_matches`` needed; the similarity refined (:func:`refine`), more than
   ``loops.temporal_refined`` inliers needed; and the points searched through the refined one,
   ``loops.temporal_matches`` needed. One more confirmation; ``loops.confirmations`` close the
   loop; ``loops.misses`` keyframes in a row without one drop it.
2. **Otherwise, the keyframe database's candidates** (:func:`slam_it.edge.database
   .loop_candidates`), each tried with its best ``loops.neighbours`` links
   (``DetectCommonRegionsFromBoW``), unless one of them is linked to the keyframe:

   - the keyframe's points matched by word to each of the group's (:func:`match_by_word`,
     ``loops.ratio``), ``loops.min_matches`` in all;
   - the similarity between them by RANSAC (:func:`slam_it.foundations.sim3.sim3_ransac`),
     ``loops.min_inliers`` needed, its samples as ORB-SLAM's ``SetRansacParameters`` counts
     them (:func:`ransac_budget`);
   - the group of the keyframe that matched most: its points searched through the similarity
     (``loops.coarse_window``, ``loops.coarse_distance``), ``loops.coarse_matches`` needed; the
     similarity refined, ``loops.min_refined`` inliers needed; searched again
     (``loops.fine_window``, ``loops.fine_distance``), ``loops.fine_matches`` needed;
   - confirmed by up to ``loops.confirmations`` of the keyframe's best ``loops.neighbours``
     links, each searching the group's points through the similarity carried to it.

   The candidate whose second search matched most is kept: closed at once with enough
   confirmations, else carried to the keyframes that follow.

**Correction** (:func:`correct`, ``CorrectLoop``): the keyframe and its links moved by the
loop's similarity, their points with them; the loop's matched points replacing the keyframe's;
the group's points fused into the moved keyframes (``SearchAndFuse``, ``loops.fuse_window``);
the essential graph optimised (:func:`essential_graph`); the loop edge added; and the whole map
bundle-adjusted (:func:`global_bundle_adjust`, ``loops.global_ba``).

**Global BA in a thread of its own** (milestone 2B's B3b): the correction runs without it, and
global BA solves a copy of the map taken then, while local mapping goes on. Its result is carried
onto the map as it has become (:func:`propagate_global_ba`, ORB-SLAM's
``RunGlobalBundleAdjustment``): the keyframes and points it solved take their result; a keyframe
made since moves with its parent in the spanning tree, and a point made since with its reference
keyframe.

**Merging** (milestone 2B's B4; the online-SLAM design, section 14): :func:`detect_merge` looks
for the place a keyframe revisits in the other maps, by detection's stages, each search taking
the map searched as well as the keyframe's own (a loop's are one); :func:`merge` brings the
active map into the one it matched, welds them, and spreads the weld (ORB-SLAM3's ``MergeLocal``,
its correction as its paper gives it).

**Similarities** are ORB-SLAM's ``S_cw``: ``[[s R, t], [0, 1]]`` carrying a world point into a
keyframe's camera, scaled; a keyframe's own pose ``T_cam_world`` is one with ``s = 1``. A search
projects through the rigid ``(R, t / s)``, as ORB-SLAM's does. Stereo and RGB-D measure their
scale, so their similarities are rigid (``mbFixScale``).

**Unlike ORB-SLAM3**, where its source does what its design does not intend:

- the matches by word over a group take each keypoint's first match, in the group's order,
  and count them; ORB-SLAM takes the last and counts the points;
- the RANSAC's second keyframe for a match is the one it was matched in, its level variance
  that observation's; ORB-SLAM's ``Sim3Solver`` inverts the test that would choose it, and so
  drops every match its most-matching keyframe does not observe;
- the refinement starts from, and is weighed against, the keyframe that matched most, whose
  frame its similarity is in; ORB-SLAM passes the candidate itself;
- a later keyframe's last search goes through the refined similarity; ORB-SLAM's goes through
  the unrefined one, its scale set to 1;
- the search of a matched keyframe's points widens to its links' links only past the new
  keyframe's links, at most ``loops.neighbours`` of each, as ORB-SLAM's design intends; its
  source adds them all;
- a group is the candidate and its best links, in their order; ORB-SLAM moves the first link
  to the end;
- a point carried behind a keyframe is never matched or fused; the moved keyframes' points
  are moved with the first of them that sees them, the new keyframe first, then its links,
  most shared first, where ORB-SLAM's order is its keyframes' addresses in memory;
- the loop's points are fused into a keyframe one at a time, each replacing the point its
  keypoint holds then: two of the loop's points that both land on one keypoint are merged.
  ORB-SLAM's ``SearchAndFuse`` defers its replacements (``vpReplacePoint``), so there the
  second finds the keypoint's first point already replaced and merges nothing (rare; B3's
  review panel, L5).
"""

from __future__ import annotations

import enum
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.backends import Crossing
from slam_it.edge import covisibility, database, residuals
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.frames import Camera
from slam_it.edge.map import Map
from slam_it.edge.options import Options, check_mode
from slam_it.foundations import camera as cameras
from slam_it.foundations import graphs, matching, ransac, se3, sim3
from slam_it.foundations.arrays import as_count
from slam_it.foundations.errors import DegenerateError, InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


# --- similarities -------------------------------------------------------------------------


def scale_of(similarity: Array) -> float:
    """Return a similarity's scale: the length of a column of ``s R``.

    Parameters
    ----------
    similarity : ndarray of float64, shape (4, 4)
        ``[[s R, t], [0, 1]]``.

    Returns
    -------
    float
        ``s``.
    """
    return float(np.linalg.norm(similarity[:3, 0]))


def rigid_of(similarity: Array) -> Array:
    """Return the rigid pose a similarity projects through: ``[[R, t / s], [0, 1]]``.

    Parameters
    ----------
    similarity : ndarray of float64, shape (4, 4)
        ``S_cw``.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        ``T_cw``, the same camera unscaled (ORB-SLAM's ``SE3(R, t / s)``).
    """
    s = scale_of(similarity)
    out = np.eye(4)
    out[:3, :3] = similarity[:3, :3] / s
    out[:3, 3] = similarity[:3, 3] / s
    return out


def _relative(to: Array, frm: Array) -> Array:
    """Return the rigid motion from one pose to another: ``T_to_w · T_frm_w⁻¹``."""
    return np.asarray(to @ sim3.inverse(frm), dtype=np.float64)


# --- searches -----------------------------------------------------------------------------


@dataclass(frozen=True)
class _InView:
    """Points that project into a keyframe: their rows, pixels and predicted levels."""

    rows: IntArray
    pixels: Array
    levels: IntArray


def _in_view(
    m: Map, camera: Camera, similarity: Array, rows: IntArray, options: Options
) -> _InView:
    """Return the points that project into a camera through a similarity, as ORB-SLAM checks them.

    In front of it, in the image, within the distances they are matchable from (no margins), and
    seen within ``tracking.view_cos_limit`` of their mean viewing direction.
    """
    pose = rigid_of(similarity)
    local = maps.to_camera(pose, m.pt_xyz[rows])
    pixels, keep = camera.project_in_view(local)
    offset = m.pt_xyz[rows] - maps.camera_centre(pose)
    distance = np.linalg.norm(offset, axis=1)
    keep &= (distance >= m.pt_min_distance[rows]) & (distance <= m.pt_max_distance[rows])
    keep &= distance > 0.0
    safe = np.where(distance > 0.0, distance, 1.0)
    keep &= maps.viewing_cos(m, rows, offset, safe) >= options.tracking.view_cos_limit
    kept = rows[keep]
    levels = (
        maps.predict_level(m, kept, distance[keep]) if len(kept) else np.zeros(0, dtype=np.int64)
    )
    return _InView(rows=kept, pixels=pixels[keep], levels=levels)


def _candidates_near(m: Map, k: int, pixel: Array, radius: float, level: int) -> IntArray:
    """Return keyframe row `k`'s keypoint rows within a window, on the level or the one below."""
    rows = np.arange(m.kf_start[k], m.kf_start[k + 1], dtype=np.int64)
    near = matching.features_in_area(
        m.kp_xy[rows], m.kp_level[rows], pixel, radius, min_level=level - 1, max_level=level
    )
    return rows[near]


@dataclass(frozen=True)
class _Searched:
    """The map a place is looked for in, and its database: the new keyframe's own, for a loop.

    In its own map the new keyframe's links are left out of the groups (``own``); another map
    holds none of them.
    """

    map: Map
    database: Database
    own: bool


def _search_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the search
    m: Map,
    camera: Camera,
    k: int,
    similarity: Array,
    points: IntArray,
    *,
    source: Map,
    window: float,
    distance: int,
    options: Options,
) -> IntArray:
    """Match points of `source` to keyframe `k` of `m` through a similarity (reference)."""
    start = int(m.kf_start[k])
    out = np.full(int(m.kf_start[k + 1]) - start, -1, dtype=np.int64)
    seen = _in_view(source, camera, similarity, points, options)
    for p, pixel, level in zip(seen.rows, seen.pixels, seen.levels.tolist(), strict=True):
        near = _candidates_near(m, k, pixel, window * m.scale_factor**level, level)
        near = near[out[near - start] < 0]
        if len(near) == 0:
            continue
        hamming = matching.distances(source.pt_descriptor[p : p + 1], m.kp_descriptor[near])[0]
        best = int(np.argmin(hamming))
        if hamming[best] <= distance:
            out[near[best] - start] = source.pt_id[p]
    return out


def _match_by_word(  # noqa: PLR0913 -- the keyframe with its map and database, the other
    m: Map, db: Database, k1: int, k2: int, *, searched: _Searched, options: Options
) -> IntArray:
    """Match keyframe `k1` of `m` to `k2` of the searched map by their points' words (reference)."""
    other = searched.map
    rows1 = np.arange(m.kf_start[k1], m.kf_start[k1 + 1], dtype=np.int64)
    rows2 = np.arange(other.kf_start[k2], other.kf_start[k2 + 1], dtype=np.int64)
    nodes1, nodes2 = db.nodes(int(m.kf_id[k1])), searched.database.nodes(int(other.kf_id[k2]))
    own1, own2 = rows1[m.kp_point[rows1] >= 0], rows2[other.kp_point[rows2] >= 0]
    found = matching.search_by_bow(
        m.kp_descriptor[own1],
        nodes1[own1 - rows1[0]] if len(rows1) else np.zeros(0, dtype=np.int64),
        other.kp_descriptor[own2],
        nodes2[own2 - rows2[0]] if len(rows2) else np.zeros(0, dtype=np.int64),
        m.kp_angle[own1],
        other.kp_angle[own2],
        matching.Options(
            max_distance=options.matching.th_low, ratio=options.loops.ratio, check_orientation=True
        ),
    )
    out = np.full(len(rows1), -1, dtype=np.int64)
    out[own1[found.pairs[:, 0]] - (rows1[0] if len(rows1) else 0)] = other.kp_point[
        own2[found.pairs[:, 1]]
    ]
    return out


def _keyframe_points(m: Map, keyframes: list[int]) -> IntArray:
    """Return the points the keyframes (rows) observe, each once, in the keyframes' order."""
    ids: list[int] = []
    taken: set[int] = set()
    for k in keyframes:
        for point_id in m.kp_point[m.keypoint_rows(k)].tolist():
            if point_id >= 0 and point_id not in taken:
                taken.add(point_id)
                ids.append(point_id)
    return m.point_rows(np.array(ids, dtype=np.int64))


def _best_links(m: Map, k: int, options: Options) -> list[int]:
    """Return a keyframe's best ``loops.neighbours`` links' rows."""
    return [int(r) for r in covisibility.links(m, k, options)[0][: options.loops.neighbours]]


def _group_points(m: Map, matched: int, linked: set[int], options: Options) -> IntArray:
    """Return the points a matched keyframe's group sees (reference)."""
    first = _best_links(m, matched, options)
    group = [*first, matched]
    if len(first) < options.loops.neighbours:
        inside = set(group)
        for row in first:
            added = 0
            for other in _best_links(m, row, options):
                if added == options.loops.neighbours:
                    break
                if other not in inside and other not in linked:
                    inside.add(other)
                    group.append(other)
                    added += 1
    return _keyframe_points(m, group)


# --- the similarity -----------------------------------------------------------------------


def ransac_budget(matches: int, options: Options) -> int:
    """Return the most samples the loop's RANSAC draws (ORB-SLAM's ``SetRansacParameters``).

    ``ceil(log(1 - p) / log(1 - e³))`` for ``e = loops.min_inliers / matches``, at most
    ``loops.ransac_iterations`` and at least 1; 1 when every match must be an inlier.

    Parameters
    ----------
    matches : int
        The matches, at least ``loops.min_inliers``.
    options : slam_it.edge.options.Options
        ``loops.min_inliers``, ``loops.confidence`` and ``loops.ransac_iterations``.

    Returns
    -------
    int
        The samples.
    """
    settings = options.loops
    if settings.min_inliers >= matches:
        return 1
    bound = ransac.required_iterations(
        settings.min_inliers / matches, sim3.SAMPLE_SIZE, settings.confidence
    )
    return max(1, min(math.ceil(bound), settings.ransac_iterations))


@dataclass(frozen=True)
class Refined:
    """A similarity refined against a keyframe's matches (:func:`refine`).

    Attributes
    ----------
    similarity : ndarray of float64, shape (4, 4)
        The refined ``S_12``.
    inliers : int
        The matches whose errors stayed within ``loops.huber_chi2`` in both images.
    """

    similarity: Array
    inliers: int


def _refine(  # noqa: PLR0913 -- the map, the two keyframes, the start, the matches, the settings
    m: Map,
    camera: Camera,
    k1: int,
    k2: int,
    *,
    other: Map,
    similarity: Array,
    matched: IntArray,
    fixed_scale: bool,
    options: Options,
) -> Refined | None:
    """Refine the similarity between keyframe `k1` of `m` and `k2` of `other` (reference)."""
    settings = options.loops
    rows1 = np.arange(m.kf_start[k1], m.kf_start[k1 + 1], dtype=np.int64)
    use = (matched >= 0) & (m.kp_point[rows1] >= 0)
    own = m.point_rows(m.kp_point[rows1[use]])
    partner = other.point_rows(matched[use])
    keep = (own >= 0) & (partner >= 0)
    rows, own, partner = rows1[use][keep], own[keep], partner[keep]
    points1 = maps.to_camera(m.kf_pose[k1], m.pt_xyz[own])
    points2 = maps.to_camera(other.kf_pose[k2], other.pt_xyz[partner])
    front = (points1[:, 2] > cameras.MIN_DEPTH) & (points2[:, 2] > cameras.MIN_DEPTH)
    rows, partner, points1, points2 = rows[front], partner[front], points1[front], points2[front]
    if len(rows) == 0:
        return None
    rows2 = np.arange(other.kf_start[k2], other.kf_start[k2 + 1], dtype=np.int64)
    pixels2 = np.full((len(rows), 2), np.nan)
    level2 = np.zeros(len(rows), dtype=np.int64)
    for i, point_id in enumerate(other.pt_id[partner].tolist()):
        at = rows2[other.kp_point[rows2] == point_id]
        if len(at):
            pixels2[i], level2[i] = other.kp_xy[at[0]], other.kp_level[at[0]]
    information = np.column_stack(
        [1.0 / residuals.sigma2(m, m.kp_level[rows]), 1.0 / residuals.sigma2(other, level2)]
    )
    pixels1 = m.kp_xy[rows]

    def run(
        start: Array, keep: Mask, huber: float | None, iterations: int
    ) -> graphs.SimilarityRefinement:
        return graphs.refine_similarity(
            start, points1[keep], points2[keep], pixels1[keep], pixels2[keep],
            information[keep], model=camera.model, intrinsics=camera.intrinsics,
            fixed_scale=fixed_scale, huber=huber,
            options=graphs.Options(max_iterations=iterations),
        )  # fmt: skip

    def within(found: graphs.SimilarityRefinement) -> Mask:
        chi2 = found.chi2
        return np.asarray((chi2[:, 0] <= settings.huber_chi2) & ~(chi2[:, 1] > settings.huber_chi2))

    everything = np.ones(len(rows), dtype=bool)
    first = run(similarity, everything, math.sqrt(settings.huber_chi2), settings.refine_iterations)
    good = within(first)
    if np.count_nonzero(good) < settings.min_kept:
        return None
    more = settings.refine_more if not np.all(good) else settings.refine_iterations
    second = run(first.similarity, good, None, more)
    return Refined(similarity=second.similarity, inliers=int(np.count_nonzero(within(second))))


# --- detection ----------------------------------------------------------------------------


class Outcome(enum.StrEnum):
    """What detection made of a keyframe."""

    SKIPPED = "skipped"
    """The map held too few keyframes to search (``loops.min_keyframes``)."""
    NONE = "none"
    """No loop: none was being confirmed, and no candidate passed."""
    STARTED = "started"
    """A candidate passed, with too few confirmations to close: carried to what follows."""
    CONFIRMED = "confirmed"
    """The loop being confirmed was confirmed again, not yet enough to close."""
    CLOSED = "closed"
    """A loop closed on this keyframe."""
    DROPPED = "dropped"
    """The loop being confirmed went unconfirmed ``loops.misses`` keyframes in a row."""
    MERGED = "merged"
    """A loop closed on the keyframe was dropped for a merge found on it (:func:`merge_first`)."""


@dataclass(frozen=True)
class Pending:
    """A loop being confirmed, carried from keyframe to keyframe (ORB-SLAM3's loop state).

    Attributes
    ----------
    matched : int
        The id of the keyframe it matched (``mpLoopMatchedKF``).
    last : int
        The id of the last keyframe that confirmed it (``mpLoopLastCurrentKF``).
    similarity : ndarray of float64, shape (4, 4)
        That keyframe's ``S_cw`` as the loop places it (``mg2oLoopSlw``).
    confirmations : int
        Its confirmations so far (``mnLoopNumCoincidences``).
    misses : int
        The keyframes in a row that failed to confirm it since (``mnLoopNumNotFound``).
    points : ndarray of int64
        The ids of the matched keyframe's group's points (``mvpLoopMPs``).
    matched_points : ndarray of int64
        For each keypoint of the last keyframe, the id of the group's point it matched, -1 for
        none (``mvpLoopMatchedMPs``).
    """

    matched: int
    last: int
    similarity: Array
    confirmations: int
    misses: int
    points: IntArray
    matched_points: IntArray

    @property
    def held(self) -> IntArray:
        """The keyframes it holds, which culling spares: the matched one and the last."""
        return np.array(sorted({self.matched, self.last}), dtype=np.int64)


@dataclass(frozen=True)
class Loop:
    """A loop closed, ready to correct.

    Attributes
    ----------
    current : int
        The id of the keyframe it closed on.
    matched : int
        The id of the keyframe it matched.
    similarity : ndarray of float64, shape (4, 4)
        The current keyframe's ``S_cw`` as the loop places it.
    points : ndarray of int64
        The ids of the matched keyframe's group's points.
    matched_points : ndarray of int64
        For each keypoint of the current keyframe, the id of the group's point it matched, -1
        for none.
    """

    current: int
    matched: int
    similarity: Array
    points: IntArray
    matched_points: IntArray


@dataclass(frozen=True)
class CandidateSummary:
    """How far one candidate got; each count None where it stopped first.

    Attributes
    ----------
    keyframe : int
        The candidate's id.
    score : float
        Its group's accumulated score.
    near : bool
        Whether its group holds a keyframe linked to the new one, so it was not tried.
    matches : int or None
        The new keyframe's points matched by word to the group's.
    best : int or None
        The id of the group's keyframe that matched most.
    inliers : int or None
        The RANSAC's inliers, if it found a similarity.
    projected : int or None
        The first search's matches through it.
    refined : int or None
        The refinement's inliers.
    reprojected : int or None
        The second search's matches through the refined similarity.
    confirmations : int or None
        The new keyframe's links that confirmed it.
    """

    keyframe: int
    score: float
    near: bool = False
    matches: int | None = None
    best: int | None = None
    inliers: int | None = None
    projected: int | None = None
    refined: int | None = None
    reprojected: int | None = None
    confirmations: int | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            Its fields, by name.
        """
        return {
            "keyframe": self.keyframe,
            "score": self.score,
            "near": self.near,
            "matches": self.matches,
            "best": self.best,
            "inliers": self.inliers,
            "projected": self.projected,
            "refined": self.refined,
            "reprojected": self.reprojected,
            "confirmations": self.confirmations,
        }


@dataclass(frozen=True)
class ConfirmationSummary:
    """How a loop being confirmed fared on a later keyframe; each count None where it stopped.

    Attributes
    ----------
    matched : int
        The id of the keyframe the loop matched.
    projected : int
        The search's matches through the similarity carried to the keyframe.
    refined : int or None
        The refinement's inliers.
    reprojected : int or None
        The search's matches through the refined similarity.
    confirmed : bool
        Whether it confirmed the loop.
    """

    matched: int
    projected: int
    refined: int | None = None
    reprojected: int | None = None
    confirmed: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            Its fields, by name.
        """
        return {
            "matched": self.matched,
            "projected": self.projected,
            "refined": self.refined,
            "reprojected": self.reprojected,
            "confirmed": self.confirmed,
        }


@dataclass(frozen=True)
class DetectionSummary:
    """How detection went on a keyframe, as a run records it.

    Attributes
    ----------
    outcome : Outcome
        What it made of the keyframe.
    confirmation : ConfirmationSummary or None
        The loop being confirmed, carried to it; None if there was none.
    sharing : int
        The keyframes outside its links that share a word with it.
    scored : int
        Those scored.
    candidates : tuple of CandidateSummary
        The candidates tried, in order.
    confirmations : int
        The loop's confirmations after this keyframe (0 if none is being confirmed).
    """

    outcome: Outcome
    confirmation: ConfirmationSummary | None = None
    sharing: int = 0
    scored: int = 0
    candidates: tuple[CandidateSummary, ...] = ()
    confirmations: int = 0

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``outcome`` (by name), ``confirmation`` (or None), ``sharing``, ``scored``,
            ``candidates`` (a list) and ``confirmations``.
        """
        return {
            "outcome": self.outcome.value,
            "confirmation": None if self.confirmation is None else self.confirmation.as_dict(),
            "sharing": self.sharing,
            "scored": self.scored,
            "candidates": [candidate.as_dict() for candidate in self.candidates],
            "confirmations": self.confirmations,
        }


@dataclass(frozen=True)
class Detection:
    """Detection's result: the loop being confirmed after it, the loop closed, how it went.

    Attributes
    ----------
    pending : Pending or None
        The loop being confirmed, carried to the keyframes that follow; None if none is.
    loop : Loop or None
        The loop closed on this keyframe, to correct; None if none closed.
    summary : DetectionSummary
        How it went.
    """

    pending: Pending | None
    loop: Loop | None
    summary: DetectionSummary


def _linked(m: Map, k: int, options: Options) -> set[int]:
    """Return a keyframe's links' rows, all of them (``GetConnectedKeyFrames``)."""
    return {int(r) for r in covisibility.links(m, k, options)[0]}


def _confirm_by_projection(  # noqa: PLR0913 -- the map, its camera, the keyframe, the loop
    m: Map,
    camera: Camera,
    k: int,
    matched: int,
    similarity: Array,
    *,
    searched: _Searched,
    options: Options,
) -> tuple[IntArray, IntArray]:
    """Search a matched keyframe's group's points in a keyframe through a similarity.

    ORB-SLAM's ``FindMatchesByProjection``: :func:`group_points` in the searched map (the
    keyframe's links left out in its own), then :func:`search_by_similarity` with
    ``loops.confirm_window`` and ``loops.confirm_distance``. Return the group's points' ids
    and each of the keyframe's keypoints' match.
    """
    linked = _linked(m, k, options) if searched.own else set()
    points = _group_points(searched.map, matched, linked, options)
    found = _search_by_similarity(
        m, camera, k, similarity, points, source=searched.map,
        window=options.loops.confirm_window, distance=options.loops.confirm_distance,
        options=options,
    )  # fmt: skip
    return searched.map.pt_id[points], found


def _matched(found: IntArray) -> int:
    """Return how many keypoints a search matched."""
    return int(np.count_nonzero(found >= 0))


def _carry(  # noqa: PLR0913 -- the map, its camera, the keyframe, the loop, the settings
    m: Map,
    camera: Camera,
    k: int,
    pending: Pending,
    *,
    searched: _Searched,
    fixed_scale: bool,
    options: Options,
) -> tuple[Pending | None, ConfirmationSummary]:
    """Try to confirm the loop being confirmed on a later keyframe (module docstring, step 1).

    The loop's last keyframe is the new keyframe's map's, its matched one the searched map's.
    Return the loop as it stands after the keyframe, None once dropped, and how it went.
    """
    settings = options.loops
    other = searched.map
    current, matched = m.kf_pose[k], maps.keyframe_row(other, pending.matched)
    last = maps.keyframe_row(m, pending.last)
    carried = _relative(current, m.kf_pose[last]) @ pending.similarity
    points, found = _confirm_by_projection(
        m, camera, k, matched, carried, searched=searched, options=options
    )
    summary = ConfirmationSummary(matched=pending.matched, projected=_matched(found))
    confirmed: tuple[Array, IntArray, IntArray] | None = None
    if summary.projected >= settings.confirm_matches:
        start = carried @ sim3.inverse(other.kf_pose[matched])
        refined = _refine(
            m, camera, k, matched, other=other, similarity=start, matched=found,
            fixed_scale=fixed_scale, options=options,
        )  # fmt: skip
        summary = replace(summary, refined=None if refined is None else refined.inliers)
        if refined is not None and refined.inliers > settings.temporal_refined:
            placed = refined.similarity @ other.kf_pose[matched]
            points, again = _confirm_by_projection(
                m, camera, k, matched, placed, searched=searched, options=options
            )
            summary = replace(summary, reprojected=_matched(again))
            if _matched(again) >= settings.temporal_matches:
                confirmed = (placed, points, again)
    if confirmed is None:
        misses = pending.misses + 1
        left = None if misses >= settings.misses else replace(pending, misses=misses)
        return left, summary
    placed, points, again = confirmed
    carried_on = Pending(
        matched=pending.matched,
        last=int(m.kf_id[k]),
        similarity=placed,
        confirmations=pending.confirmations + 1,
        misses=0,
        points=points,
        matched_points=again,
    )
    return carried_on, replace(summary, confirmed=True)


@dataclass(frozen=True)
class _Found:
    """A candidate that passed every test: where it places the loop and what it matched."""

    best: int
    similarity: Array
    points: IntArray
    matched_points: IntArray
    reprojected: int
    confirmations: int


def _merged_by_word(  # noqa: PLR0913 -- the keyframe with its map and database, the group
    m: Map, db: Database, k: int, group: list[int], *, searched: _Searched, options: Options
) -> tuple[IntArray, IntArray, int]:
    """Match a keyframe by word to each of a group's keyframes, and merge the matches.

    The group is the searched map's. Each keypoint takes its first match in the group's order
    whose point no earlier keypoint took. Return each keypoint's point id and the row of the
    keyframe it was matched in (-1 where none), and the row of the group's keyframe that
    matched most (ties: the first).
    """
    size = int(m.kf_start[k + 1] - m.kf_start[k])
    merged = np.full(size, -1, dtype=np.int64)
    source = np.full(size, -1, dtype=np.int64)
    used: set[int] = set()
    best, most = group[0], -1
    for g in group:
        found = _match_by_word(m, db, k, g, searched=searched, options=options)
        if _matched(found) > most:
            best, most = g, _matched(found)
        for i in np.flatnonzero(found >= 0).tolist():
            point_id = int(found[i])
            if merged[i] < 0 and point_id not in used:
                merged[i], source[i] = point_id, g
                used.add(point_id)
    return merged, source, best


def _similarity_by_ransac(  # noqa: PLR0913 -- the map, its camera, the matches, the settings
    m: Map,
    camera: Camera,
    k: int,
    best: int,
    merged: IntArray,
    *,
    other: Map,
    source: IntArray,
    fixed_scale: bool,
    options: Options,
) -> tuple[Array | None, int | None]:
    """Find the similarity from the best keyframe's frame to the keyframe's by RANSAC.

    Each merged match gives the keyframe's own point in its frame and the matched point in the
    best keyframe's (of `other`), both in front, each with its keypoint's level variance (the
    matched point's in the keyframe it was matched in). Return ``S_kb`` and its inliers, or
    None and the inliers it found (None if it did not run) when it falls short of
    ``loops.min_inliers``.
    """
    settings = options.loops
    rows = m.kf_start[k] + np.flatnonzero(merged >= 0)
    own = m.point_rows(m.kp_point[rows])
    partner = other.point_rows(merged[merged >= 0])
    points1 = maps.to_camera(m.kf_pose[k], m.pt_xyz[own])
    points2 = maps.to_camera(other.kf_pose[best], other.pt_xyz[partner])
    level2 = np.zeros(len(rows), dtype=np.int64)
    for i, (point_id, g) in enumerate(
        zip(merged[merged >= 0].tolist(), source[merged >= 0].tolist(), strict=True)
    ):
        at = np.flatnonzero(other.kp_point[other.keypoint_rows(g)] == point_id)
        level2[i] = other.kp_level[other.kf_start[g] + at[0]]
    front = (points1[:, 2] > cameras.MIN_DEPTH) & (points2[:, 2] > cameras.MIN_DEPTH)
    count = int(np.count_nonzero(front))
    if count < max(settings.min_inliers, sim3.SAMPLE_SIZE):
        return None, None
    run = ransac.Options(
        threshold=math.sqrt(settings.chi2),
        confidence=settings.confidence,
        max_iterations=ransac_budget(count, options),
        seed=options.seed,
    )
    try:
        found = sim3.sim3_ransac(
            points1[front], points2[front], residuals.sigma2(m, m.kp_level[rows[front]]),
            residuals.sigma2(other, level2[front]), run, model=camera.model,
            intrinsics=camera.intrinsics, fixed_scale=fixed_scale,
        )  # fmt: skip
    except DegenerateError:
        return None, 0
    inliers = found.summary.inlier_count
    if inliers < settings.min_inliers:
        return None, inliers
    return found.state.model, inliers


def _confirmations(  # noqa: PLR0913 -- the map, its camera, the keyframe, the loop
    m: Map,
    camera: Camera,
    k: int,
    best: int,
    similarity: Array,
    *,
    searched: _Searched,
    options: Options,
) -> int:
    """Count the keyframe's best links that confirm the loop, at most ``loops.confirmations``."""
    count = 0
    for j in _best_links(m, k, options):
        if count == options.loops.confirmations:
            break
        carried = _relative(m.kf_pose[j], m.kf_pose[k]) @ similarity
        _, found = _confirm_by_projection(
            m, camera, j, best, carried, searched=searched, options=options
        )
        if _matched(found) >= options.loops.confirm_matches:
            count += 1
    return count


def _place(  # noqa: PLR0913 -- the map, its camera, the two keyframes, the start, the record
    m: Map,
    camera: Camera,
    k: int,
    best: int,
    model: Array,
    *,
    other: Map,
    summary: CandidateSummary,
    fixed_scale: bool,
    options: Options,
) -> tuple[tuple[Array, IntArray, IntArray] | None, CandidateSummary]:
    """Place a candidate's loop: search, refine, search again (module docstring, step 2).

    The best keyframe's (of `other`) group's points searched through ``S_kb`` (`model`), the
    similarity refined against those matches, and the points searched through the refined one.
    Return the keyframe's ``S_cw`` in `other`'s world, the group's points' rows and the second
    search's matches, or None where a test failed; and the summary with each count reached.
    """
    settings = options.loops
    points = _keyframe_points(other, [*_best_links(other, best, options), best])
    coarse = _search_by_similarity(
        m, camera, k, model @ other.kf_pose[best], points, source=other,
        window=settings.coarse_window, distance=settings.coarse_distance, options=options,
    )  # fmt: skip
    summary = replace(summary, projected=_matched(coarse))
    if _matched(coarse) < settings.coarse_matches:
        return None, summary
    refined = _refine(
        m, camera, k, best, other=other, similarity=model, matched=coarse,
        fixed_scale=fixed_scale, options=options,
    )  # fmt: skip
    summary = replace(summary, refined=None if refined is None else refined.inliers)
    if refined is None or refined.inliers < settings.min_refined:
        return None, summary
    placed = refined.similarity @ other.kf_pose[best]
    fine = _search_by_similarity(
        m, camera, k, placed, points, source=other,
        window=settings.fine_window, distance=settings.fine_distance, options=options,
    )  # fmt: skip
    summary = replace(summary, reprojected=_matched(fine))
    if _matched(fine) < settings.fine_matches:
        return None, summary
    return (placed, points, fine), summary


def _try_candidate(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the try
    m: Map,
    db: Database,
    camera: Camera,
    k: int,
    candidate: tuple[int, float],
    *,
    searched: _Searched,
    linked: set[int],
    fixed_scale: bool,
    options: Options,
) -> tuple[CandidateSummary, _Found | None]:
    """Try one of the database's candidates, a keyframe of the searched map (docstring, step 2).

    A group holding one of `linked`, the keyframe's links in its own map, is passed over.
    """
    settings = options.loops
    other = searched.map
    keyframe, score = candidate
    c = maps.keyframe_row(other, keyframe)
    group = [c, *_best_links(other, c, options)]
    summary = CandidateSummary(keyframe=keyframe, score=score)
    if linked & set(group):
        return replace(summary, near=True), None
    merged, source, best = _merged_by_word(m, db, k, group, searched=searched, options=options)
    summary = replace(summary, matches=_matched(merged), best=int(other.kf_id[best]))
    if _matched(merged) < settings.min_matches:
        return summary, None
    model, inliers = _similarity_by_ransac(
        m, camera, k, best, merged, other=other, source=source, fixed_scale=fixed_scale,
        options=options,
    )  # fmt: skip
    summary = replace(summary, inliers=inliers)
    if model is None:
        return summary, None
    placed, summary = _place(
        m, camera, k, best, model, other=other, summary=summary, fixed_scale=fixed_scale,
        options=options,
    )  # fmt: skip
    if placed is None:
        return summary, None
    similarity, points, fine = placed
    confirmations = _confirmations(
        m, camera, k, best, similarity, searched=searched, options=options
    )
    found = _Found(
        best=best,
        similarity=similarity,
        points=other.pt_id[points],
        matched_points=fine,
        reprojected=_matched(fine),
        confirmations=confirmations,
    )
    return replace(summary, confirmations=confirmations), found


def _search(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the settings
    m: Map, db: Database, camera: Camera, k: int, *, fixed_scale: bool, options: Options
) -> tuple[_Found | None, DetectionSummary]:
    """Try the database's candidates; return the best that passed, and how the search went."""
    candidates = database.loop_candidates(db, m, int(m.kf_id[k]), options)
    own = _Searched(m, db, own=True)
    linked = _linked(m, k, options)
    tried: list[CandidateSummary] = []
    best: _Found | None = None
    for keyframe, score in zip(
        candidates.keyframes.tolist(), candidates.scores.tolist(), strict=True
    ):
        summary, found = _try_candidate(
            m, db, camera, k, (keyframe, score), searched=own, linked=linked,
            fixed_scale=fixed_scale, options=options,
        )  # fmt: skip
        tried.append(summary)
        if found is not None and (best is None or found.reprojected > best.reprojected):
            best = found
    searched = DetectionSummary(
        outcome=Outcome.NONE,
        sharing=candidates.sharing,
        scored=candidates.scored,
        candidates=tuple(tried),
    )
    return best, searched


def _detect(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the state
    m: Map,
    db: Database,
    camera: Camera,
    keyframe: int,
    pending: Pending | None,
    *,
    mode: str,
    options: Options,
) -> Detection:
    """Look for a loop at a newly mapped keyframe (reference)."""
    k = maps.keyframe_row(m, keyframe)
    fixed_scale = mode != "monocular"
    settings = options.loops
    held = 0 if pending is None else pending.confirmations
    if m.keyframes < settings.min_keyframes:
        return Detection(pending, None, DetectionSummary(Outcome.SKIPPED, confirmations=held))
    confirmation: ConfirmationSummary | None = None
    dropped = False
    if pending is not None:
        carried, confirmation = _carry(
            m, camera, k, pending, searched=_Searched(m, db, own=True), fixed_scale=fixed_scale,
            options=options,
        )  # fmt: skip
        if confirmation.confirmed and carried is not None:
            confirmed = DetectionSummary(Outcome.CONFIRMED, confirmation)
            return _settle(carried, confirmed, options=options)
        dropped = carried is None
        pending = carried
    found, searched = _search(m, db, camera, k, fixed_scale=fixed_scale, options=options)
    searched = replace(searched, confirmation=confirmation)
    if found is None:
        outcome = Outcome.DROPPED if dropped else Outcome.NONE
        held = 0 if pending is None else pending.confirmations
        return Detection(pending, None, replace(searched, outcome=outcome, confirmations=held))
    started = Pending(
        matched=int(m.kf_id[found.best]),
        last=keyframe,
        similarity=found.similarity,
        confirmations=found.confirmations,
        misses=0,
        points=found.points,
        matched_points=found.matched_points,
    )
    return _settle(started, replace(searched, outcome=Outcome.STARTED), options=options)


def _settle(state: Pending, summary: DetectionSummary, *, options: Options) -> Detection:
    """Close a loop with enough confirmations, carry it with fewer, keep none with none.

    `state` is the loop as the keyframe left it, its last the keyframe; `summary` says what
    happened if it is carried. ORB-SLAM keeps no loop with no confirmation to carry.
    """
    summary = replace(summary, confirmations=state.confirmations)
    if state.confirmations >= options.loops.confirmations:
        loop = Loop(
            current=state.last,
            matched=state.matched,
            similarity=state.similarity,
            points=state.points,
            matched_points=state.matched_points,
        )
        return Detection(None, loop, replace(summary, outcome=Outcome.CLOSED))
    if state.confirmations == 0:
        return Detection(None, None, replace(summary, outcome=Outcome.NONE, confirmations=0))
    return Detection(state, None, summary)


# --- merge detection ----------------------------------------------------------------------


@dataclass(frozen=True)
class Merging:
    """A merge being confirmed: the matched map's number, and the common region's state.

    Attributes
    ----------
    map : int
        The number of the map it matched.
    pending : Pending
        The region as a loop being confirmed holds it: its matched keyframe that map's, its
        last the active map's.
    """

    map: int
    pending: Pending


@dataclass(frozen=True)
class Merge:
    """A merge found, ready to make (:func:`merge`).

    Attributes
    ----------
    map : int
        The number of the map it matched.
    loop : Loop
        The common region as a closed loop holds it: its current keyframe the active map's,
        its matched one that map's, its similarity the current keyframe's ``S_cw`` in that
        map's world (scaled by the two maps' scale ratio in monocular).
    """

    map: int
    loop: Loop


@dataclass(frozen=True)
class MergeDetection:
    """Merge detection's result, as :class:`Detection` is loop detection's.

    Attributes
    ----------
    pending : Merging or None
        The merge being confirmed, carried to the keyframes that follow; None if none is.
    merge : Merge or None
        The merge found on this keyframe; None if none was.
    summary : DetectionSummary
        How it went, as loop detection's says; each candidate's keyframe an id in its map.
    maps : tuple of int
        Each candidate's map number, in the summary's order.
    """

    pending: Merging | None
    merge: Merge | None
    summary: DetectionSummary
    maps: tuple[int, ...] = ()


def _settle_merge(
    number: int,
    state: Pending,
    summary: DetectionSummary,
    *,
    maps: tuple[int, ...],
    options: Options,
) -> MergeDetection:
    """Settle a merge as :func:`_settle` settles a loop, its map's number kept with it."""
    settled = _settle(state, summary, options=options)
    return MergeDetection(
        pending=None if settled.pending is None else Merging(number, settled.pending),
        merge=None if settled.loop is None else Merge(number, settled.loop),
        summary=settled.summary,
        maps=maps,
    )


def _search_merge(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the others
    m: Map,
    db: Database,
    camera: Camera,
    k: int,
    others: dict[int, database.Indexed],
    *,
    fixed_scale: bool,
    options: Options,
) -> tuple[tuple[int, _Found] | None, DetectionSummary, tuple[int, ...]]:
    """Try the other maps' candidates; return the best that passed with its map, and the search.

    Each candidate is tried as a loop's (:func:`_try_candidate`) against its own map, which
    holds none of the keyframe's links.
    """
    candidates = database.merge_candidates(
        db.vector(int(m.kf_id[k])), list(others.values()), options
    )
    tried: list[CandidateSummary] = []
    best: tuple[int, _Found] | None = None
    for number, keyframe, score in zip(
        candidates.maps.tolist(),
        candidates.keyframes.tolist(),
        candidates.scores.tolist(),
        strict=True,
    ):
        other = others[number]
        summary, found = _try_candidate(
            m, db, camera, k, (keyframe, score),
            searched=_Searched(other.map, other.database, own=False), linked=set(),
            fixed_scale=fixed_scale, options=options,
        )  # fmt: skip
        tried.append(summary)
        if found is not None and (best is None or found.reprojected > best[1].reprojected):
            best = (number, found)
    searched = DetectionSummary(
        outcome=Outcome.NONE,
        sharing=candidates.sharing,
        scored=candidates.scored,
        candidates=tuple(tried),
    )
    return best, searched, tuple(candidates.maps.tolist())


def _detect_merge(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the state
    m: Map,
    db: Database,
    camera: Camera,
    keyframe: int,
    *,
    others: dict[int, database.Indexed],
    pending: Merging | None,
    mode: str,
    options: Options,
) -> MergeDetection:
    """Look for another map at a newly mapped keyframe (reference)."""
    k = maps.keyframe_row(m, keyframe)
    fixed_scale = mode != "monocular"
    settings = options.loops
    held = 0 if pending is None else pending.pending.confirmations
    if m.keyframes < settings.min_keyframes:
        skipped = DetectionSummary(Outcome.SKIPPED, confirmations=held)
        return MergeDetection(pending, None, skipped)
    confirmation: ConfirmationSummary | None = None
    dropped = False
    if pending is not None:
        other = others[pending.map]
        carried, confirmation = _carry(
            m, camera, k, pending.pending,
            searched=_Searched(other.map, other.database, own=False), fixed_scale=fixed_scale,
            options=options,
        )  # fmt: skip
        if confirmation.confirmed and carried is not None:
            confirmed = DetectionSummary(Outcome.CONFIRMED, confirmation)
            return _settle_merge(pending.map, carried, confirmed, maps=(), options=options)
        dropped = carried is None
        pending = None if carried is None else Merging(pending.map, carried)
    found, searched, tried = _search_merge(
        m, db, camera, k, others, fixed_scale=fixed_scale, options=options
    )
    searched = replace(searched, confirmation=confirmation)
    if found is None:
        outcome = Outcome.DROPPED if dropped else Outcome.NONE
        held = 0 if pending is None else pending.pending.confirmations
        summary = replace(searched, outcome=outcome, confirmations=held)
        return MergeDetection(pending, None, summary, tried)
    number, best = found
    started = Pending(
        matched=int(others[number].map.kf_id[best.best]),
        last=keyframe,
        similarity=best.similarity,
        confirmations=best.confirmations,
        misses=0,
        points=best.points,
        matched_points=best.matched_points,
    )
    began = replace(searched, outcome=Outcome.STARTED)
    return _settle_merge(number, started, began, maps=tried, options=options)


def _merge_first(loop: Detection, merging: MergeDetection) -> Detection:
    """Give a merge priority over a loop closed on its keyframe (reference)."""
    if merging.merge is None or loop.loop is None:
        return loop
    return Detection(None, None, replace(loop.summary, outcome=Outcome.MERGED))


# --- correction ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CorrectionSummary:
    """How a loop's correction went, as a run records it.

    Attributes
    ----------
    current : int
        The id of the keyframe the loop closed on.
    matched : int
        The id of the keyframe it matched.
    similarity : ndarray of float64, shape (4, 4)
        The current keyframe's ``S_cw`` as the loop placed it.
    corrected : int
        The keyframes moved by the loop's similarity: the current one and its links.
    moved : int
        The points moved with them.
    replaced : int
        The current keyframe's points the loop's matched points replaced, or joined.
    fused : int
        The loop's points fused into the moved keyframes.
    links : int
        The links new across the loop.
    edges : int
        The essential graph's edges.
    essential : slam_it.foundations.graphs.Summary
        Its solve.
    global_ba : slam_it.foundations.graphs.Summary or None
        Global BA's solve; None with ``loops.global_ba`` off.
    """

    current: int
    matched: int
    similarity: Array
    corrected: int
    moved: int
    replaced: int
    fused: int
    links: int
    edges: int
    essential: graphs.Summary
    global_ba: graphs.Summary | None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            Its fields, the similarity as rows, each solve as
            :func:`slam_it.foundations.graphs.Summary`'s fields (or None).
        """
        return {
            "current": self.current,
            "matched": self.matched,
            "similarity": self.similarity.tolist(),
            "corrected": self.corrected,
            "moved": self.moved,
            "replaced": self.replaced,
            "fused": self.fused,
            "links": self.links,
            "edges": self.edges,
            "essential": _solve_record(self.essential),
            "global_ba": None if self.global_ba is None else _solve_record(self.global_ba),
        }


def _solve_record(summary: graphs.Summary) -> dict[str, Any]:
    """Return a solve's summary as JSON values."""
    return {
        "initial_error": summary.initial_error,
        "final_error": summary.final_error,
        "iterations": summary.iterations,
        "termination": summary.termination.value,
    }


@dataclass(frozen=True)
class Corrected:
    """A loop corrected: the map, and how it went.

    Attributes
    ----------
    map : slam_it.edge.map.Map
        The map, its keyframes and points moved onto the loop, the loop edge added.
    summary : CorrectionSummary
        How it went.
    """

    map: Map
    summary: CorrectionSummary


def _fuse_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the fusion
    m: Map,
    camera: Camera,
    k: int,
    similarity: Array,
    point_ids: IntArray,
    *,
    options: Options,
) -> tuple[Map, int]:
    """Fuse points into a keyframe through a similarity (reference)."""
    keyframe = int(m.kf_id[k])
    fused = 0
    for point_id in point_ids.tolist():
        k = maps.keyframe_row(m, keyframe)
        p = int(m.point_rows(np.array([point_id], dtype=np.int64))[0])
        if p < 0 or np.any(m.kp_point[m.keypoint_rows(k)] == point_id):
            continue
        seen = _in_view(m, camera, similarity, np.array([p], dtype=np.int64), options)
        if len(seen.rows) == 0:
            continue
        level = int(seen.levels[0])
        near = _candidates_near(
            m, k, seen.pixels[0], options.loops.fuse_window * m.scale_factor**level, level
        )
        if len(near) == 0:
            continue
        hamming = matching.distances(m.pt_descriptor[p : p + 1], m.kp_descriptor[near])[0]
        best = int(np.argmin(hamming))
        if hamming[best] > options.matching.th_low:
            continue
        held = int(m.kp_point[near[best]])
        if held >= 0:
            m = maps.replace_point(m, lose=held, keep=point_id)
        else:
            kp_point = m.kp_point.copy()
            kp_point[near[best]] = point_id
            m = maps.refresh_points(replace(m, kp_point=kp_point), np.array([p]))
        fused += 1
    return m, fused


def _move_keyframes(
    m: Map, moved: list[int], corrected: dict[int, Array]
) -> tuple[Map, dict[int, int], int]:
    """Move keyframes by their corrected similarities, each point with the first that sees it.

    Return the map, each moved point's id to the row of the keyframe that moved it, and the
    points moved.
    """
    poses = m.kf_pose.copy()
    xyz = m.pt_xyz.copy()
    moved_by: dict[int, int] = {}
    for r in moved:
        carry = sim3.inverse(corrected[r]) @ m.kf_pose[r]
        ids = m.kp_point[m.keypoint_rows(r)]
        for point_id, p in zip(ids.tolist(), m.point_rows(ids).tolist(), strict=True):
            if point_id < 0 or point_id in moved_by:
                continue
            xyz[p] = carry[:3, :3] @ m.pt_xyz[p] + carry[:3, 3]
            moved_by[point_id] = r
    for r in moved:
        poses[r] = rigid_of(corrected[r])
    shifted = replace(m, kf_pose=poses, pt_xyz=xyz)
    rows = shifted.point_rows(np.array(sorted(moved_by), dtype=np.int64))
    return maps.refresh_points(shifted, rows), moved_by, len(moved_by)


def _replace_matched(m: Map, k: int, matched_points: IntArray) -> tuple[Map, int]:
    """Replace the keyframe's points by the loop's it matched, or give its free keypoints them."""
    keyframe = int(m.kf_id[k])
    count = 0
    for i, point_id in enumerate(matched_points.tolist()):
        k = maps.keyframe_row(m, keyframe)
        row = int(m.kf_start[k]) + i
        if point_id < 0 or m.point_rows(np.array([point_id]))[0] < 0:
            continue
        if np.any(m.kp_point[m.keypoint_rows(k)] == point_id):
            continue  # the keyframe sees it already
        held = int(m.kp_point[row])
        if held >= 0:
            m = maps.replace_point(m, lose=held, keep=point_id)
        else:
            kp_point = m.kp_point.copy()
            kp_point[row] = point_id
            m = maps.refresh_points(
                replace(m, kp_point=kp_point), m.point_rows(np.array([point_id]))
            )
        count += 1
    return m, count


def _weight(m: Map, a: int, b: int) -> int:
    """Return the points two keyframes (rows) share."""
    own = m.kp_point[m.keypoint_rows(a)]
    theirs = m.kp_point[m.keypoint_rows(b)]
    return len(np.intersect1d(own[own >= 0], theirs[theirs >= 0]))


@dataclass(frozen=True)
class _Graph:
    """The essential graph's edges, as rows, and each one's measured relative similarity."""

    edges: list[tuple[int, int]]
    relatives: list[Array]


def _essential_edges(  # noqa: PLR0913 -- the map, the loop's state, the settings
    m: Map,
    start: list[Array],
    before: list[Array],
    loop_links: dict[int, set[int]],
    ends: tuple[int, int],
    *,
    options: Options,
) -> _Graph:
    """Collect the essential graph's edges (ORB-SLAM's ``OptimizeEssentialGraph``).

    `start` holds each keyframe's ``S_iw`` as the graph starts, `before` its pose before the
    correction (the same for one the loop did not move); `ends` the current and matched rows.
    """
    current, matched = ends
    edges: list[tuple[int, int]] = []
    relatives: list[Array] = []
    inserted: set[tuple[int, int]] = set()

    def add(i: int, j: int, poses: list[Array]) -> None:
        edges.append((i, j))
        relatives.append(poses[i] @ sim3.inverse(poses[j]))

    for i in sorted(loop_links):
        for j in sorted(loop_links[i]):
            if (i, j) != (current, matched) and _weight(m, i, j) < options.loops.essential_points:
                continue
            add(i, j, start)
            inserted.add((min(i, j), max(i, j)))
    for i in range(m.keyframes):
        parent = int(m.keyframe_rows(m.kf_parent[i : i + 1])[0])
        if parent >= 0:
            add(i, parent, before)
        for older, newer in m.loop_edges.tolist():
            if newer == m.kf_id[i]:
                add(i, maps.keyframe_row(m, older), before)
        linked, weights = covisibility.links(m, i, options)
        for n, weight in zip(linked.tolist(), weights.tolist(), strict=True):
            if (
                weight < options.loops.essential_points
                or n == parent
                or m.kf_parent[n] == m.kf_id[i]
                or m.kf_id[n] > m.kf_id[i]
                or (min(i, n), max(i, n)) in inserted
            ):
                continue
            add(i, n, before)
    return _Graph(edges=edges, relatives=relatives)


def essential_graph(  # noqa: PLR0913 -- the map, the loop's state, the settings
    m: Map,
    start: list[Array],
    before: list[Array],
    loop_links: dict[int, set[int]],
    ends: tuple[int, int],
    *,
    fixed: list[int],
    fixed_scale: bool,
    options: Options,
) -> tuple[list[Array], int, graphs.Summary]:
    """Optimise the essential graph from the loop's start (ORB-SLAM's ``OptimizeEssentialGraph``).

    A node per keyframe, starting from `start`; an edge for each of: the loop's new links
    sharing ``loops.essential_points`` points, and the current keyframe's to the matched one
    whatever it shares, each measured between the starting similarities; and, measured between
    the poses before the correction, each keyframe's to its parent, its loop edges to older
    keyframes, and its links sharing ``loops.essential_points`` points to older keyframes but its
    parent and children. Unit information, the `fixed` keyframes held (a loop's matched one, a
    merge's welding window), ``loops.essential_iterations`` of Levenberg-Marquardt from
    ``loops.essential_damping`` (:func:`slam_it.foundations.graphs.optimize_pose_graph`), on
    Sim(3) but in stereo and RGB-D.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    start : list of ndarray
        Each keyframe's ``S_iw`` the graph starts from, by row.
    before : list of ndarray
        Each one's pose before the correction, by row.
    loop_links : dict of int to set of int
        Each moved keyframe's (row) links new across the loop.
    ends : tuple of int
        The current and the matched keyframes' rows.
    fixed : list of int
        The rows held fixed, at least one.
    fixed_scale : bool
        Optimise SE(3), not Sim(3).
    options : slam_it.edge.options.Options
        The ``loops`` settings.

    Returns
    -------
    tuple
        Each keyframe's optimised ``S_iw``, the edges, and the solve's summary.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `start` or `before` is not one 4x4 matrix per keyframe, an end or a link is not a
        keyframe row, or the two ends are one keyframe.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while solving.
    """
    count = m.keyframes
    for name, poses in (("start", start), ("before", before)):
        if len(poses) != count or any(np.shape(pose) != (4, 4) for pose in poses):
            raise InvalidInputError(f"{name}: expected one 4x4 matrix per keyframe ({count})")
    rows = [*ends, *loop_links, *(n for linked in loop_links.values() for n in linked)]
    if any(not 0 <= int(r) < count for r in rows):
        raise InvalidInputError(f"ends, loop_links: expected keyframe rows in [0, {count})")
    if ends[0] == ends[1]:
        raise InvalidInputError("ends: the current and the matched keyframes must differ")
    graph = _essential_edges(m, start, before, loop_links, ends, options=options)
    solved = graphs.optimize_pose_graph(
        np.array([sim3.inverse(s) for s in start]),
        np.array(graph.edges, dtype=np.int64).reshape(-1, 2),
        np.array(graph.relatives).reshape(-1, 4, 4),
        fixed,
        similarity=not fixed_scale,
        options=graphs.Options(
            max_iterations=options.loops.essential_iterations,
            initial_damping=options.loops.essential_damping,
        ),
    )
    return [sim3.inverse(pose) for pose in solved.poses], len(graph.edges), solved.summary


def _global_bundle_adjust(
    m: Map, camera: Camera, options: Options, stop: Callable[[], bool] | None = None
) -> tuple[Map, graphs.Summary]:
    """Bundle-adjust every keyframe and point (reference)."""
    rows = np.flatnonzero(m.kp_point >= 0)
    adjusted = graphs.bundle_adjust(
        np.array([sim3.inverse(pose) for pose in m.kf_pose]),
        m.pt_xyz,
        m.keypoint_keyframe()[rows],
        m.point_rows(m.kp_point[rows]),
        m.kp_xy[rows],
        m.kp_right[rows],
        1.0 / residuals.sigma2(m, m.kp_level[rows]),
        model=camera.model,
        intrinsics=camera.intrinsics,
        bf=camera.focal * camera.baseline,
        fixed=np.flatnonzero(m.kf_id == 0),
        options=graphs.Options(max_iterations=options.loops.global_iterations),
        stop=stop,
    )
    poses = np.array([sim3.inverse(pose) for pose in adjusted.poses]).reshape(-1, 4, 4)
    out = replace(m, kf_pose=poses, pt_xyz=adjusted.points)
    return maps.refresh_points(out, np.arange(out.points)), adjusted.summary


def _correct(m: Map, camera: Camera, loop: Loop, *, mode: str, options: Options) -> Corrected:
    """Correct a loop closed at a keyframe (reference)."""
    k, matched = maps.keyframe_row(m, loop.current), maps.keyframe_row(m, loop.matched)
    fixed_scale = mode != "monocular"
    links = [int(r) for r in covisibility.links(m, k, options)[0]]
    moved_rows = [k, *links]
    before = list(m.kf_pose.copy())
    corrected = {k: loop.similarity}
    to_world = sim3.inverse(m.kf_pose[k])
    for r in links:
        corrected[r] = m.kf_pose[r] @ to_world @ loop.similarity
    previous = {r: _linked(m, r, options) for r in moved_rows}
    m, moved_by, moved = _move_keyframes(m, moved_rows, corrected)
    m, replaced = _replace_matched(m, k, loop.matched_points)
    # Moving, replacing and fusing change points and the moved keyframes' poses, but never add,
    # remove or reorder a keyframe: every keyframe keeps its row from here to the graph, so the
    # rows found above still name them.
    fused = 0
    for r in moved_rows:
        m, count = _fuse_by_similarity(m, camera, r, corrected[r], loop.points, options=options)
        fused += count
    moving = set(moved_rows)
    loop_links = {r: _linked(m, r, options) - previous[r] - moving for r in moved_rows}
    start = list(m.kf_pose.copy())
    for r in moved_rows:
        start[r] = corrected[r]
    optimised, edges, essential = essential_graph(
        m, start, before, loop_links, (k, matched), fixed=[matched], fixed_scale=fixed_scale,
        options=options,
    )  # fmt: skip
    m = _move_by_graph(m, start, optimised, moved_by)
    m = maps.add_loop_edge(m, loop.matched, loop.current)
    ba: graphs.Summary | None = None
    if options.loops.global_ba:
        m, ba = _global_bundle_adjust(m, camera, options)
    summary = CorrectionSummary(
        current=loop.current,
        matched=loop.matched,
        similarity=loop.similarity,
        corrected=len(moved_rows),
        moved=moved,
        replaced=replaced,
        fused=fused,
        links=sum(len(v) for v in loop_links.values()),
        edges=edges,
        essential=essential,
        global_ba=ba,
    )
    return Corrected(map=m, summary=summary)


@dataclass(frozen=True)
class Propagated:
    """A global BA's result carried onto the map as it has become (:func:`propagate_global_ba`).

    Attributes
    ----------
    map : slam_it.edge.map.Map
        The map, moved.
    adjusted : int
        Its keyframes the global BA solved, set to their result.
    propagated : int
        Its keyframes made since, moved with their parents.
    points : int
        Its points the global BA solved, set to their result.
    carried : int
        Its points made since, moved with their reference keyframes.
    """

    map: Map
    adjusted: int
    propagated: int
    points: int
    carried: int


@dataclass(frozen=True)
class GlobalSummary:
    """How a global BA in a thread of its own went, as a run records it on the frame it ended on.

    Attributes
    ----------
    generation : int
        Its generation: the loops corrected before it in the run.
    applied : bool
        Whether its result was carried onto the map (:func:`propagate_global_ba`); not if a
        newer loop superseded it, or the map was lost, first.
    solve : slam_it.foundations.graphs.Summary
        Its solve.
    propagation : Propagated or None
        How its result was carried, when it was.
    """

    generation: int
    applied: bool
    solve: graphs.Summary
    propagation: Propagated | None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``generation``, ``applied``, ``solve``, and the keyframes and points carried each
            way (``adjusted``, ``propagated``, ``points``, ``carried``; 0 when not applied).
        """
        moved = self.propagation
        return {
            "generation": self.generation,
            "applied": self.applied,
            "solve": _solve_record(self.solve),
            "adjusted": 0 if moved is None else moved.adjusted,
            "propagated": 0 if moved is None else moved.propagated,
            "points": 0 if moved is None else moved.points,
            "carried": 0 if moved is None else moved.carried,
        }


def propagate_global_ba(now: Map, solved: Map, *, backend: str = "python") -> Propagated:
    """Carry a global BA's result, solved on an earlier copy of the map, onto the map as it is now.

    ORB-SLAM's ``RunGlobalBundleAdjustment``, after its solve: walking the spanning tree from its
    roots (the map's first keyframe, and any keyframe without a parent), each keyframe the BA
    solved takes its solved pose, and each keyframe made since moves with its parent, its pose
    relative to the parent kept (``T_child' = T_child T_parent^-1 T_parent'``); each point the BA
    solved takes its solved position, and each made since moves with its reference keyframe
    (``X' = T_ref'^-1 T_ref X``). A keyframe the walk does not reach, or a point whose reference
    keyframe is gone, stays where it is. Every point is refreshed after
    (:func:`slam_it.edge.map.refresh_points`).

    Parameters
    ----------
    now : slam_it.edge.map.Map
        The map as it is now.
    solved : slam_it.edge.map.Map
        The copy the global BA solved, moved to its result (:func:`global_bundle_adjust`).
    backend : {"python", "cpp", "rust"}, optional
        The implementation to run (default: the Python reference).

    Returns
    -------
    Propagated
        The map moved, and how many keyframes and points each way.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If either map has no keyframe, or `solved` cannot be an earlier copy of `now`: a map
        gives its ids in order, so a copy has given no more keyframe or point ids than it.
    """
    if not now.keyframes or not solved.keyframes:
        raise InvalidInputError(
            f"now, solved: expected keyframes, got {now.keyframes} and {solved.keyframes}"
        )
    if solved.next_kf_id > now.next_kf_id or solved.next_pt_id > now.next_pt_id:
        raise InvalidInputError(
            "solved: not an earlier copy of now, its next ids "
            f"({solved.next_kf_id} keyframe, {solved.next_pt_id} point) past now's "
            f"({now.next_kf_id}, {now.next_pt_id})"
        )

    def decode(raw: Crossing, codes: backends.Codes) -> Propagated:
        counts = {name: int(raw[name]) for name in ("adjusted", "propagated", "points", "carried")}
        return Propagated(map=maps.decode(raw["map"], codes), **counts)

    return backends.call(
        backend,
        __name__,
        "propagate_global_ba",
        reference=lambda: _propagate_global_ba(now, solved),
        arguments={"now": now.as_dict(), "solved": solved.as_dict()},
        decode=decode,
    )


def _propagate_global_ba(now: Map, solved: Map) -> Propagated:
    """Carry a global BA's result onto the map as it is now (reference)."""
    new_poses = now.kf_pose.copy()
    solved_rows = solved.keyframe_rows(now.kf_id)
    rows_of = {int(keyframe): k for k, keyframe in enumerate(now.kf_id)}
    children: dict[int, list[int]] = {}
    for k, parent in enumerate(now.kf_parent.tolist()):
        children.setdefault(int(parent), []).append(k)
    adjusted = propagated = 0
    walk = list(children.get(-1, []))
    while walk:
        k = walk.pop(0)
        parent = int(now.kf_parent[k])
        if solved_rows[k] >= 0:
            new_poses[k] = solved.kf_pose[solved_rows[k]]
            adjusted += 1
        elif parent in rows_of:
            p = rows_of[parent]
            new_poses[k] = now.kf_pose[k] @ se3.inverse(now.kf_pose[p]) @ new_poses[p]
            propagated += 1
        walk.extend(children.get(int(now.kf_id[k]), []))
    xyz = now.pt_xyz.copy()
    from_solve = solved.point_rows(now.pt_id)
    held = from_solve >= 0
    xyz[held] = solved.pt_xyz[from_solve[held]]
    carried = 0
    for p in np.flatnonzero(~held).tolist():
        reference = rows_of.get(int(now.pt_reference_kf[p]))
        if reference is None:
            continue
        to_camera = now.kf_pose[reference]
        camera_xyz = to_camera[:3, :3] @ xyz[p] + to_camera[:3, 3]
        to_world = se3.inverse(new_poses[reference])
        xyz[p] = to_world[:3, :3] @ camera_xyz + to_world[:3, 3]
        carried += 1
    moved = replace(now, kf_pose=new_poses, pt_xyz=xyz)
    return Propagated(
        map=maps.refresh_points(moved, np.arange(moved.points)),
        adjusted=adjusted,
        propagated=propagated,
        points=int(np.count_nonzero(held)),
        carried=carried,
    )


def _move_by_graph(
    m: Map, start: list[Array], optimised: list[Array], moved_by: dict[int, int]
) -> Map:
    """Set each keyframe to its optimised pose, and move each point with its keyframe.

    A point moved by the loop goes with the keyframe that moved it; any other with its reference
    keyframe (ORB-SLAM's ``mnCorrectedReference``, else ``GetReferenceKeyFrame``).
    """
    xyz = m.pt_xyz.copy()
    references = m.keyframe_rows(m.pt_reference_kf)
    for p, point_id in enumerate(m.pt_id.tolist()):
        r = moved_by.get(point_id)
        if r is None:
            r = int(references[p])
            if r < 0:
                continue
        carry = sim3.inverse(optimised[r]) @ start[r]
        xyz[p] = carry[:3, :3] @ m.pt_xyz[p] + carry[:3, 3]
    poses = np.array([rigid_of(s) for s in optimised]).reshape(-1, 4, 4)
    out = replace(m, kf_pose=poses, pt_xyz=xyz)
    return maps.refresh_points(out, np.arange(out.points))


# --- merging ------------------------------------------------------------------------------


@dataclass(frozen=True)
class WeldSummary:
    """How a merge's welding BA went, as a run records it.

    Attributes
    ----------
    keyframes : int
        The keyframes optimised: both welding windows', or the active side's alone where the
        matched window was held.
    held : int
        The keyframes held fixed: the matched map's outside the windows that observe their
        points, or else the matched window.
    points : int
        The points optimised: every point the windows observe.
    observations : int
        The observations weighed: those of the optimised and held keyframes.
    outliers : int
        Those failing the chi-square test, or behind their camera, after the first solve, and
        left out of the second.
    forgotten : int
        Those failing it after the second, forgotten (:func:`slam_it.edge.map.forget_observations`).
    first : slam_it.foundations.graphs.Summary
        The first solve, with a Huber loss.
    second : slam_it.foundations.graphs.Summary or None
        The second, with none; None if no observation was left for it.
    """

    keyframes: int
    held: int
    points: int
    observations: int
    outliers: int
    forgotten: int
    first: graphs.Summary
    second: graphs.Summary | None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            Its counts, and each solve as :func:`slam_it.foundations.graphs.Summary`'s fields
            (or None).
        """
        return {
            "keyframes": self.keyframes,
            "held": self.held,
            "points": self.points,
            "observations": self.observations,
            "outliers": self.outliers,
            "forgotten": self.forgotten,
            "first": _solve_record(self.first),
            "second": None if self.second is None else _solve_record(self.second),
        }


@dataclass(frozen=True)
class MergeSummary:
    """How a merge went, as a run records it.

    Attributes
    ----------
    map : int
        The number of the map the active one was merged into.
    current : int
        The id, in the merged map, of the active map's keyframe the merge was found on.
    matched : int
        The id of the keyframe it matched.
    similarity : ndarray of float64, shape (4, 4)
        The current keyframe's ``S_cw`` in the matched map's world, as the merge placed it.
    window, matched_window : int
        The welding windows' keyframes, on each side.
    fused : int
        The matched window's points fused into the active window's keyframes.
    weld : WeldSummary
        The welding BA.
    edges : int
        The essential graph's edges.
    essential : slam_it.foundations.graphs.Summary
        Its solve.
    global_ba : slam_it.foundations.graphs.Summary or None
        Global BA's solve; None with ``loops.global_ba`` off.
    """

    map: int
    current: int
    matched: int
    similarity: Array
    window: int
    matched_window: int
    fused: int
    weld: WeldSummary
    edges: int
    essential: graphs.Summary
    global_ba: graphs.Summary | None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            Its fields, the similarity as rows, the welding BA as :meth:`WeldSummary.as_dict`,
            each solve as :func:`slam_it.foundations.graphs.Summary`'s fields (or None).
        """
        return {
            "map": self.map,
            "current": self.current,
            "matched": self.matched,
            "similarity": self.similarity.tolist(),
            "window": self.window,
            "matched_window": self.matched_window,
            "fused": self.fused,
            "weld": self.weld.as_dict(),
            "edges": self.edges,
            "essential": _solve_record(self.essential),
            "global_ba": None if self.global_ba is None else _solve_record(self.global_ba),
        }


@dataclass(frozen=True)
class Merged:
    """Two maps merged: the matched map with the active one in it, and how it went.

    Attributes
    ----------
    map : slam_it.edge.map.Map
        The merged map: the matched map's keyframes and points, ids and world, then the
        active map's, each id moved on by an offset.
    keyframe_offset : int
        What each of the active map's keyframe ids, live or archived, became: the id plus this
        (the matched map's next keyframe id).
    point_offset : int
        The same for its point ids (the matched map's next point id).
    summary : MergeSummary
        How it went.
    """

    map: Map
    keyframe_offset: int
    point_offset: int
    summary: MergeSummary


def _scaled(pose: Array, scale: float) -> Array:
    """Return a pose scaled into a similarity: ``[[s R, s t], [0, 1]]``."""
    out = pose.copy()
    out[:3, :] *= scale
    return out


def _moved_into(m: Map, to_world: Array) -> Map:
    """Carry a map into another world by a similarity ``G`` on its own: ``p`` goes to ``G p``.

    Each keyframe's pose becomes the rigid one through which it sees its points where it saw
    them, scaled (:func:`rigid_of` of ``T G⁻¹``); its archive with it, each culled keyframe's
    pose relative to its parent scaled into the new world's units. Every point observed is
    refreshed.
    """
    back = sim3.inverse(to_world)
    scale = scale_of(back)
    turn, shift = to_world[:3, :3], to_world[:3, 3]
    relative = m.gone_kf_relative.copy()
    relative[:, :3, 3] /= scale
    out = replace(
        m,
        kf_pose=np.array([rigid_of(pose @ back) for pose in m.kf_pose]).reshape(-1, 4, 4),
        pt_xyz=m.pt_xyz @ turn.T + shift,
        gone_kf_pose=np.array([rigid_of(pose @ back) for pose in m.gone_kf_pose]).reshape(-1, 4, 4),
        gone_pt_xyz=m.gone_pt_xyz @ turn.T + shift,
        gone_kf_relative=relative,
    )
    seen = out.point_rows(np.unique(out.kp_point[out.kp_point >= 0]))
    return maps.refresh_points(out, seen[seen >= 0])


def _offset(ids: IntArray, offset: int) -> IntArray:
    """Return ids moved on by an offset, -1 (none) kept."""
    return np.where(ids >= 0, ids + offset, ids).astype(np.int64)


def _absorb(into: Map, other: Map, keyframe_offset: int, point_offset: int) -> Map:
    """Return `into` with `other`'s keyframes, keypoints, points and archive after its own.

    Every id of `other`'s, live or archived, moves on by its offset; `other` must already be in
    `into`'s world (:func:`_moved_into`).
    """
    kf, pt = keyframe_offset, point_offset
    tables: dict[str, Any] = {
        name: np.concatenate([getattr(into, name), getattr(other, name)])
        for name in (
            "kf_time", "kf_pose", "kp_xy", "kp_level", "kp_angle", "kp_descriptor", "kp_bearing",
            "kp_depth", "kp_right", "pt_xyz", "pt_descriptor", "pt_normal", "pt_min_distance",
            "pt_max_distance", "pt_visible", "pt_found", "gone_pt_xyz", "gone_pt_how",
            "gone_kf_time", "gone_kf_pose", "gone_kf_relative",
        )
    }  # fmt: skip
    return replace(
        into,
        **tables,
        kf_id=np.concatenate([into.kf_id, other.kf_id + kf]),
        kf_parent=np.concatenate([into.kf_parent, _offset(other.kf_parent, kf)]),
        kf_start=np.concatenate([into.kf_start, other.kf_start[1:] + into.kf_start[-1]]),
        kp_point=np.concatenate([into.kp_point, _offset(other.kp_point, pt)]),
        pt_id=np.concatenate([into.pt_id, other.pt_id + pt]),
        pt_first_kf=np.concatenate([into.pt_first_kf, other.pt_first_kf + kf]),
        pt_reference_kf=np.concatenate([into.pt_reference_kf, other.pt_reference_kf + kf]),
        next_kf_id=kf + other.next_kf_id,
        next_pt_id=pt + other.next_pt_id,
        gone_pt_id=np.concatenate([into.gone_pt_id, other.gone_pt_id + pt]),
        gone_kf_id=np.concatenate([into.gone_kf_id, other.gone_kf_id + kf]),
        gone_kf_parent=np.concatenate([into.gone_kf_parent, _offset(other.gone_kf_parent, kf)]),
        loop_edges=np.concatenate([into.loop_edges, other.loop_edges + kf]),
        merge_edges=np.concatenate([into.merge_edges, other.merge_edges + kf]),
    )


def _rehang(m: Map, current: int, matched: int) -> Map:
    """Re-root the active side's spanning tree at its current keyframe, hung under the matched.

    Each parent on the path from the current keyframe up to its tree's root becomes the child
    of its child, and the current keyframe's parent becomes the matched one (ORB-SLAM3's
    ``ChangeParent`` chain in ``MergeLocal``): the old root is an ordinary keyframe after, and
    the merged map's root is the matched map's own.
    """
    parent_of = dict(zip(m.kf_id.tolist(), m.kf_parent.tolist(), strict=True))
    chain = [current]
    while parent_of[chain[-1]] >= 0:
        chain.append(parent_of[chain[-1]])
    parents = m.kf_parent.copy()
    rows = m.keyframe_rows(np.array(chain, dtype=np.int64))
    for child, row in zip(chain[:-1], rows[1:].tolist(), strict=True):
        parents[row] = child
    parents[rows[0]] = matched
    return replace(m, kf_parent=parents)


def _weld_window(m: Map, k: int, options: Options) -> list[int]:
    """Return a welding window's keyframe rows (ORB-SLAM3's ``MergeLocal``, visual).

    The keyframe and its best ``loops.weld_neighbours`` links; while it holds fewer than that,
    each member's best ``loops.weld_widen`` links not yet in it, at most ``loops.weld_rounds``
    times. In the order taken in.
    """
    settings = options.loops
    window = [k, *covisibility.links(m, k, options)[0][: settings.weld_neighbours].tolist()]
    for _ in range(settings.weld_rounds):
        if len(window) >= settings.weld_neighbours:
            break
        inside, added = set(window), []
        for r in window:
            for other in covisibility.links(m, r, options)[0][: settings.weld_widen].tolist():
                if other not in inside:
                    inside.add(other)
                    added.append(other)
        if not added:
            break
        window += added
    return [int(r) for r in window]


@dataclass(frozen=True)
class _Bundle:
    """A welding BA's problem: its cameras and points, and each observation of them."""

    cameras: list[int]
    fixed: list[int]
    points: IntArray
    rows: IntArray
    camera: IntArray
    point: IntArray
    xy: Array
    right: Array
    variance: Array
    limits: Array


def _weld_problem(
    m: Map, windows: tuple[list[int], list[int]], keyframe_offset: int, options: Options
) -> _Bundle:
    """Gather a welding BA's problem (the paper's: ORB-SLAM3, section VI-B, step 3).

    Both windows' keyframes free, with every point they observe; the matched map's keyframes
    outside them that observe those points held fixed (ORB-SLAM3's source holds the matched
    window instead, which serves here only when no such keyframe exists). The active map's
    keyframes outside the window are left out.
    """
    window, matched_window = windows
    free = sorted(set(window) | set(matched_window))
    points = m.observed_by(np.array(free, dtype=np.int64))
    keyframe_of = m.keypoint_keyframe()
    observing = np.flatnonzero(np.isin(m.kp_point, m.pt_id[points]))
    outside = sorted({int(r) for r in keyframe_of[observing]} - set(free))
    held = [r for r in outside if int(m.kf_id[r]) < keyframe_offset]
    cameras = free + held
    fixed = held or sorted(matched_window)
    rows = observing[np.isin(keyframe_of[observing], cameras)]
    index = {r: i for i, r in enumerate(cameras)}
    return _Bundle(
        cameras=cameras,
        fixed=[index[r] for r in fixed],
        points=points,
        rows=rows,
        camera=np.array([index[int(r)] for r in keyframe_of[rows]], dtype=np.int64),
        point=np.searchsorted(points, m.point_rows(m.kp_point[rows])).astype(np.int64),
        xy=m.kp_xy[rows],
        right=m.kp_right[rows],
        variance=residuals.sigma2(m, m.kp_level[rows]),
        limits=residuals.thresholds(m.kp_right[rows], options),
    )


def _weld_solve(  # noqa: PLR0913 -- the problem, its state, which observations, the solve
    problem: _Bundle,
    camera: Camera,
    state: tuple[Array, Array],
    use: Mask,
    *,
    robust: bool,
    iterations: int,
) -> tuple[Array, Array, graphs.Summary]:
    """Solve the welding BA on the observations `use` marks, from `state` (poses, points).

    Only the cameras and points those observations reach take part, with the held cameras;
    the rest keep their state.
    """
    poses, xyz = state
    cameras = np.unique(np.concatenate([problem.camera[use], problem.fixed])).astype(np.int64)
    points = np.unique(problem.point[use])
    result = graphs.bundle_adjust(
        poses[cameras], xyz[points], np.searchsorted(cameras, problem.camera[use]),
        np.searchsorted(points, problem.point[use]), problem.xy[use], problem.right[use],
        1.0 / problem.variance[use], model=camera.model, intrinsics=camera.intrinsics,
        bf=camera.focal * camera.baseline, fixed=np.searchsorted(cameras, problem.fixed),
        huber=np.sqrt(problem.limits[use]) if robust else None,
        options=graphs.Options(max_iterations=iterations),
    )  # fmt: skip
    poses, xyz = poses.copy(), xyz.copy()
    poses[cameras], xyz[points] = result.poses, result.points
    return poses, xyz, result.summary


def _weld_errors(problem: _Bundle, camera: Camera, poses: Array, xyz: Array) -> Array:
    """Return each observation's chi-square error at `poses` (``T_world_cam``) and `xyz`."""
    to_camera = np.array([sim3.inverse(pose) for pose in poses]).reshape(-1, 4, 4)
    rotation, shift = to_camera[problem.camera, :3, :3], to_camera[problem.camera, :3, 3]
    local = np.einsum("nij,nj->ni", rotation, xyz[problem.point]) + shift
    return residuals.chi2(camera, local, problem.xy, problem.right, problem.variance)


def _weld(
    m: Map,
    camera: Camera,
    windows: tuple[list[int], list[int]],
    keyframe_offset: int,
    options: Options,
) -> tuple[Map, WeldSummary]:
    """Bundle-adjust the welding windows (ORB-SLAM3's welding BA, as its paper gives it).

    :func:`_weld_problem`'s problem, solved as ORB-SLAM's local BA solves its own: a Huber loss
    at each observation's chi-square threshold's root (:func:`slam_it.edge.residuals.thresholds`)
    for ``adjustment.iterations``; the observations then failing the test, or behind their
    camera, left out; ``adjustment.refine_iterations`` more with no robust loss; the observations
    still failing it forgotten, and the points refreshed.
    """
    problem = _weld_problem(m, windows, keyframe_offset, options)
    state = (
        np.array([sim3.inverse(m.kf_pose[r]) for r in problem.cameras]).reshape(-1, 4, 4),
        m.pt_xyz[problem.points].copy(),
    )
    everything = np.ones(len(problem.rows), dtype=bool)
    poses, xyz, first = _weld_solve(
        problem, camera, state, everything, robust=True, iterations=options.adjustment.iterations
    )
    inliers = _weld_errors(problem, camera, poses, xyz) <= problem.limits
    second: graphs.Summary | None = None
    if np.any(inliers):
        poses, xyz, second = _weld_solve(
            problem, camera, (poses, xyz), inliers, robust=False,
            iterations=options.adjustment.refine_iterations,
        )  # fmt: skip
    failing = ~(_weld_errors(problem, camera, poses, xyz) <= problem.limits)
    kf_pose, pt_xyz = m.kf_pose.copy(), m.pt_xyz.copy()
    held = set(problem.fixed)
    for i, r in enumerate(problem.cameras):
        if i not in held:
            kf_pose[r] = sim3.inverse(poses[i])
    pt_xyz[problem.points] = xyz
    out = maps.refresh_points(replace(m, kf_pose=kf_pose, pt_xyz=pt_xyz), problem.points)
    out = maps.forget_observations(out, problem.rows[failing])
    summary = WeldSummary(
        keyframes=len(problem.cameras) - len(held),
        held=len(held),
        points=len(problem.points),
        observations=len(problem.rows),
        outliers=int(np.count_nonzero(~inliers)),
        forgotten=int(np.count_nonzero(failing)),
        first=first,
        second=second,
    )
    return out, summary


def merge_transform(active: Map, found: Merge) -> Array:
    """Return ``G``, the similarity carrying a merge's active map into the matched map's world.

    ``G = S⁻¹ T``: `found`'s similarity places the current keyframe in the matched map's world,
    ``T`` is its pose in the active map's; a point ``p`` of the active map goes to ``G p``.

    Parameters
    ----------
    active : slam_it.edge.map.Map
        The active map, holding the merge's current keyframe.
    found : Merge
        The merge.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        ``G``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the active map does not hold the current keyframe.
    """
    region = found.loop
    placed = active.kf_pose[maps.keyframe_row(active, region.current)]
    return np.asarray(sim3.inverse(region.similarity) @ placed, dtype=np.float64)


def _merge(  # noqa: PLR0913 -- the two maps, the camera, the merge, the settings
    active: Map, matched: Map, camera: Camera, found: Merge, *, mode: str, options: Options
) -> Merged:
    """Merge the active map into the matched one (reference)."""
    region = found.loop
    fixed_scale = mode != "monocular"
    scale = 1.0 if fixed_scale else scale_of(region.similarity)
    to_matched = merge_transform(active, found)
    keyframe_offset, point_offset = matched.next_kf_id, matched.next_pt_id
    m = _absorb(matched, _moved_into(active, to_matched), keyframe_offset, point_offset)
    current = region.current + keyframe_offset
    m = _rehang(m, current, region.matched)
    k, km = maps.keyframe_row(m, current), maps.keyframe_row(m, region.matched)
    window, matched_window = _weld_window(m, k, options), _weld_window(m, km, options)
    points = m.pt_id[_keyframe_points(m, matched_window)]
    fused = 0
    for r in window:  # fusion erases points, never keyframes: the rows hold
        m, count = _fuse_by_similarity(m, camera, r, m.kf_pose[r], points, options=options)
        fused += count
    before = m.kf_pose.copy()
    m, weld = _weld(m, camera, (window, matched_window), keyframe_offset, options)
    absorbed = (m.kf_id >= keyframe_offset).tolist()
    # The active side's nodes carry its own units, as its edges measured before were.
    start = [_scaled(p, scale) if a else p for p, a in zip(m.kf_pose, absorbed, strict=True)]
    prior = [_scaled(p, scale) if a else p for p, a in zip(before, absorbed, strict=True)]
    welded = sorted(set(window) | set(matched_window))
    optimised, edges, essential = essential_graph(
        m, start, prior, {}, (k, km), fixed=welded, fixed_scale=fixed_scale, options=options
    )
    m = _move_by_graph(m, start, optimised, {})
    m = maps.add_merge_edge(m, region.matched, current)
    ba: graphs.Summary | None = None
    if options.loops.global_ba:
        m, ba = _global_bundle_adjust(m, camera, options)
    summary = MergeSummary(
        map=found.map,
        current=current,
        matched=region.matched,
        similarity=region.similarity,
        window=len(window),
        matched_window=len(matched_window),
        fused=fused,
        weld=weld,
        edges=edges,
        essential=essential,
        global_ba=ba,
    )
    return Merged(m, keyframe_offset, point_offset, summary)


# --- the backends -------------------------------------------------------------------------


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.loops`` submodule provides.

    The map, its database, the camera and the settings cross as dicts (their ``as_dict``); a
    similarity as its 4x4 matrix; a :class:`Pending` and a :class:`Loop` as dicts
    (``_pending_as_dict``, ``_loop_as_dict``); the mode by name. Results cross back as dicts:
    a detection's ``pending`` and ``loop`` (each a dict or None) and ``summary``, its outcome a
    code in ``codes()["loop_outcome"]``; a correction's ``map`` and ``summary``, each solve's
    termination a code in ``codes()["graphs_termination"]``. Merging's: the other maps as a
    list of :meth:`slam_it.edge.database.Indexed.as_dict`; a :class:`Merging` and a
    :class:`Merge` as ``map`` with ``pending`` or ``loop``; a detection given to
    :func:`merge_first` with its summary's :meth:`DetectionSummary.as_dict`, its outcome by name;
    a merge detection back as a detection is, with ``maps``; a merge back as ``map``,
    ``keyframe_offset``, ``point_offset`` and ``summary``.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def search_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the search
        self,
        *,
        m: Crossing,
        camera: Crossing,
        k: int,
        similarity: Array,
        points: IntArray,
        window: float,
        distance: int,
        options: Crossing,
    ) -> IntArray:
        """Match points to a keyframe's keypoints through a similarity."""
        ...

    def match_by_word(
        self, *, m: Crossing, database: Crossing, k1: int, k2: int, options: Crossing
    ) -> IntArray:
        """Match two keyframes' points by word."""
        ...

    def group_points(
        self, *, m: Crossing, matched: int, linked: IntArray, options: Crossing
    ) -> IntArray:
        """Return the points a matched keyframe's group sees."""
        ...

    def refine(  # noqa: PLR0913 -- the map, the two keyframes, the start, the matches
        self,
        *,
        m: Crossing,
        camera: Crossing,
        k1: int,
        k2: int,
        similarity: Array,
        matched: IntArray,
        fixed_scale: bool,
        options: Crossing,
    ) -> Crossing | None:
        """Refine the similarity between two keyframes against their matches."""
        ...

    def fuse_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the fusion
        self,
        *,
        m: Crossing,
        camera: Crossing,
        k: int,
        similarity: Array,
        point_ids: IntArray,
        options: Crossing,
    ) -> Crossing:
        """Fuse points into a keyframe through a similarity."""
        ...

    def global_bundle_adjust(self, *, m: Crossing, camera: Crossing, options: Crossing) -> Crossing:
        """Bundle-adjust every keyframe and point."""
        ...

    def propagate_global_ba(self, *, now: Crossing, solved: Crossing) -> Crossing:
        """Carry a global BA's result onto the map as it has become."""
        ...

    def detect(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the state
        self,
        *,
        m: Crossing,
        database: Crossing,
        camera: Crossing,
        keyframe: int,
        pending: Crossing | None,
        mode: str,
        options: Crossing,
    ) -> Crossing:
        """Look for a loop at a newly mapped keyframe."""
        ...

    def correct(
        self, *, m: Crossing, camera: Crossing, loop: Crossing, mode: str, options: Crossing
    ) -> Crossing:
        """Correct a loop closed at a keyframe."""
        ...

    def detect_merge(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the state
        self,
        *,
        m: Crossing,
        database: Crossing,
        camera: Crossing,
        keyframe: int,
        others: list[Crossing],
        pending: Crossing | None,
        mode: str,
        options: Crossing,
    ) -> Crossing:
        """Look for another map at a newly mapped keyframe."""
        ...

    def merge_first(self, *, loop: Crossing, merging: Crossing) -> Crossing:
        """Give a merge found on a keyframe priority over a loop closed on it."""
        ...

    def merge(  # noqa: PLR0913 -- the two maps, the camera, the merge, the settings
        self,
        *,
        active: Crossing,
        matched: Crossing,
        camera: Crossing,
        found: Crossing,
        mode: str,
        options: Crossing,
    ) -> Crossing:
        """Merge the active map into the one it matched."""
        ...


def codes() -> dict[str, list[str]]:
    """Return the module's enum names in code order, as a compiled backend's ``codes()`` does.

    Returns
    -------
    dict of str to list of str
        ``loop_outcome`` (:class:`Outcome`) and ``graphs_termination``
        (:class:`slam_it.foundations.graphs.Termination`).
    """
    return {
        "loop_outcome": [o.value for o in Outcome],
        "graphs_termination": [t.value for t in graphs.Termination],
    }


def _similarity(similarity: ArrayLike, name: str = "similarity") -> Array:
    """Check a similarity: a finite 4x4 ``[[s R, t], [0, 1]]``, ``s > 0``."""
    out = np.asarray(similarity, dtype=np.float64)
    if out.shape != (4, 4) or not np.all(np.isfinite(out)):
        raise InvalidInputError(f"{name}: expected a finite 4x4 similarity")
    scale = np.cbrt(np.linalg.det(out[:3, :3]))
    rotation = out[:3, :3] / scale if scale > 0.0 else out[:3, :3]
    if scale <= 0.0 or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
        raise InvalidInputError(f"{name}: expected s R with s > 0 and R a rotation")
    if not np.array_equal(out[3], [0.0, 0.0, 0.0, 1.0]):
        raise InvalidInputError(f"{name}: expected a last row of 0 0 0 1")
    return out


def _ids(values: ArrayLike, name: str) -> IntArray:
    """Check a vector of ids: integers, at least -1, refused under the argument's own name."""
    try:
        return covisibility.as_points(values)
    except InvalidInputError:
        raise InvalidInputError(f"{name}: expected a vector of ids of at least -1") from None


def _keypoint_ids(m: Map, k: int, values: ArrayLike, name: str) -> IntArray:
    """Check a keyframe's keypoint-by-keypoint ids: one per keypoint, at least -1."""
    out = _ids(values, name)
    if len(out) != int(m.kf_start[k + 1] - m.kf_start[k]):
        raise InvalidInputError(f"{name}: expected one id per keypoint of keyframe row {k}")
    return out


def _pending_as_dict(pending: Pending) -> Crossing:
    """Return a loop being confirmed as it crosses the bindings."""
    return {
        "matched": pending.matched,
        "last": pending.last,
        "similarity": pending.similarity,
        "confirmations": pending.confirmations,
        "misses": pending.misses,
        "points": pending.points,
        "matched_points": pending.matched_points,
    }


def _loop_as_dict(loop: Loop) -> Crossing:
    """Return a closed loop as it crosses the bindings."""
    return {
        "current": loop.current,
        "matched": loop.matched,
        "similarity": loop.similarity,
        "points": loop.points,
        "matched_points": loop.matched_points,
    }


def _checked_ids(raw: Any, name: str) -> IntArray:  # noqa: ANN401 -- a compiled backend's value
    """Return a compiled backend's id vector, checked."""
    return backends.checked_array(raw, np.int64, 1, name)


def _checked_similarity(raw: Any, name: str) -> Array:  # noqa: ANN401 -- a compiled backend's value
    """Return a compiled backend's similarity, checked."""
    return backends.checked_array(raw, np.float64, 2, name)


def _decode_pending(raw: Crossing | None) -> Pending | None:
    """Rebuild a loop being confirmed from a compiled backend's dict."""
    if raw is None:
        return None
    return Pending(
        matched=int(raw["matched"]),
        last=int(raw["last"]),
        similarity=_checked_similarity(raw["similarity"], "pending similarity"),
        confirmations=int(raw["confirmations"]),
        misses=int(raw["misses"]),
        points=_checked_ids(raw["points"], "pending points"),
        matched_points=_checked_ids(raw["matched_points"], "pending matched_points"),
    )


def _decode_loop(raw: Crossing | None) -> Loop | None:
    """Rebuild a closed loop from a compiled backend's dict."""
    if raw is None:
        return None
    return Loop(
        current=int(raw["current"]),
        matched=int(raw["matched"]),
        similarity=_checked_similarity(raw["similarity"], "loop similarity"),
        points=_checked_ids(raw["points"], "loop points"),
        matched_points=_checked_ids(raw["matched_points"], "loop matched_points"),
    )


def _optional(raw: Any) -> int | None:  # noqa: ANN401 -- a compiled backend's value
    """Return an optional count as an int or None."""
    return None if raw is None else int(raw)


def _decode_detection(raw: Crossing, codes: backends.Codes) -> Detection:
    """Rebuild :class:`Detection` from a compiled backend's dict."""
    return Detection(
        pending=_decode_pending(raw["pending"]),
        loop=_decode_loop(raw["loop"]),
        summary=_decode_summary(raw["summary"], codes),
    )


def _decode_merge_detection(raw: Crossing, codes: backends.Codes) -> MergeDetection:
    """Rebuild :class:`MergeDetection` from a compiled backend's dict."""
    pending, found = raw["pending"], raw["merge"]
    carried = _decode_pending(None if pending is None else pending["pending"])
    region = _decode_loop(None if found is None else found["loop"])
    return MergeDetection(
        pending=None
        if pending is None or carried is None
        else Merging(int(pending["map"]), carried),
        merge=None if found is None or region is None else Merge(int(found["map"]), region),
        summary=_decode_summary(raw["summary"], codes),
        maps=tuple(_checked_ids(raw["maps"], "merge detection maps").tolist()),
    )


def _decode_summary(summary: Crossing, codes: backends.Codes) -> DetectionSummary:
    """Rebuild :class:`DetectionSummary` from a compiled backend's dict."""
    carried = summary["confirmation"]
    confirmation = (
        None
        if carried is None
        else ConfirmationSummary(
            matched=int(carried["matched"]),
            projected=int(carried["projected"]),
            refined=_optional(carried["refined"]),
            reprojected=_optional(carried["reprojected"]),
            confirmed=bool(carried["confirmed"]),
        )
    )
    candidates = tuple(
        CandidateSummary(
            keyframe=int(c["keyframe"]),
            score=float(c["score"]),
            near=bool(c["near"]),
            matches=_optional(c["matches"]),
            best=_optional(c["best"]),
            inliers=_optional(c["inliers"]),
            projected=_optional(c["projected"]),
            refined=_optional(c["refined"]),
            reprojected=_optional(c["reprojected"]),
            confirmations=_optional(c["confirmations"]),
        )
        for c in summary["candidates"]
    )
    outcome = Outcome(backends.decode_name(codes, "loop_outcome", summary["outcome"]))
    return DetectionSummary(
        outcome=outcome,
        confirmation=confirmation,
        sharing=int(summary["sharing"]),
        scored=int(summary["scored"]),
        candidates=candidates,
        confirmations=int(summary["confirmations"]),
    )


def _decode_merged(raw: Crossing, codes: backends.Codes) -> Merged:
    """Rebuild :class:`Merged` from a compiled backend's dict."""
    s, weld = raw["summary"], raw["summary"]["weld"]
    second, ba = weld["second"], s["global_ba"]
    return Merged(
        map=maps.decode(raw["map"], codes),
        keyframe_offset=int(raw["keyframe_offset"]),
        point_offset=int(raw["point_offset"]),
        summary=MergeSummary(
            map=int(s["map"]),
            current=int(s["current"]),
            matched=int(s["matched"]),
            similarity=_checked_similarity(s["similarity"], "merge similarity"),
            window=int(s["window"]),
            matched_window=int(s["matched_window"]),
            fused=int(s["fused"]),
            weld=WeldSummary(
                keyframes=int(weld["keyframes"]),
                held=int(weld["held"]),
                points=int(weld["points"]),
                observations=int(weld["observations"]),
                outliers=int(weld["outliers"]),
                forgotten=int(weld["forgotten"]),
                first=graphs.summary_from_backend(weld["first"], codes),
                second=None if second is None else graphs.summary_from_backend(second, codes),
            ),
            edges=int(s["edges"]),
            essential=graphs.summary_from_backend(s["essential"], codes),
            global_ba=None if ba is None else graphs.summary_from_backend(ba, codes),
        ),
    )


def _detection_as_dict(detection: Detection) -> Crossing:
    """Return a loop detection's result as it crosses the bindings, its outcome by name."""
    return {
        "pending": None if detection.pending is None else _pending_as_dict(detection.pending),
        "loop": None if detection.loop is None else _loop_as_dict(detection.loop),
        "summary": detection.summary.as_dict(),
    }


def _merging_as_dict(merging: Merging | None) -> Crossing | None:
    """Return a merge being confirmed as it crosses the bindings."""
    if merging is None:
        return None
    return {"map": merging.map, "pending": _pending_as_dict(merging.pending)}


def _merge_detection_as_dict(found: MergeDetection) -> Crossing:
    """Return merge detection's result as it crosses the bindings, its outcome by name."""
    return {
        "pending": _merging_as_dict(found.pending),
        "merge": None if found.merge is None else _merge_as_dict(found.merge),
        "summary": found.summary.as_dict(),
        "maps": np.array(found.maps, dtype=np.int64),
    }


def _merge_as_dict(found: Merge) -> Crossing:
    """Return a merge found as it crosses the bindings."""
    return {"map": found.map, "loop": _loop_as_dict(found.loop)}


def _decode_corrected(raw: Crossing, codes: backends.Codes) -> Corrected:
    """Rebuild :class:`Corrected` from a compiled backend's dict."""
    s = raw["summary"]
    ba = s["global_ba"]
    return Corrected(
        map=maps.decode(raw["map"], codes),
        summary=CorrectionSummary(
            current=int(s["current"]),
            matched=int(s["matched"]),
            similarity=_checked_similarity(s["similarity"], "correction similarity"),
            corrected=int(s["corrected"]),
            moved=int(s["moved"]),
            replaced=int(s["replaced"]),
            fused=int(s["fused"]),
            links=int(s["links"]),
            edges=int(s["edges"]),
            essential=graphs.summary_from_backend(s["essential"], codes),
            global_ba=None if ba is None else graphs.summary_from_backend(ba, codes),
        ),
    )


def search_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the search
    m: Map,
    camera: Camera,
    k: int,
    similarity: ArrayLike,
    points: ArrayLike,
    *,
    window: float,
    distance: int,
    options: Options,
    backend: str = "python",
) -> IntArray:
    """Match points to a keyframe's keypoints through a similarity (``SearchByProjection``).

    Each point in view (in front of the camera, in the image, within the distances it is
    matchable from, and seen within ``tracking.view_cos_limit`` of its mean viewing direction),
    in order, is compared with the keyframe's keypoints not yet matched in this search, within
    ``window`` times its predicted level's scale, on that level or the one below; the nearest
    by Hamming distance (ties: the lower row) takes it if within `distance`. No ratio test, no
    rotation check, as ORB-SLAM's similarity search.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    k : int
        The keyframe's row.
    similarity : array_like, shape (4, 4)
        Its ``S_cw``, through which the points are carried.
    points : array_like of int
        The points' rows, in the order they are searched.
    window : float
        The window's half-width, in pixels at level 0.
    distance : int
        The largest Hamming distance a match may have.
    options : slam_it.edge.options.Options
        ``tracking.view_cos_limit``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (N,)
        The id of the point each of the keyframe's keypoints matched, -1 where none.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `k` is not a keyframe row, the similarity is not one, a point row is out of range,
        `window` is not finite and positive, or `distance` is outside ``[0,
        slam_it.foundations.matching.MAX_DISTANCE]``.
    """
    row = maps.as_keyframe_row(m, k)
    chosen = _similarity(similarity)
    rows = maps.as_point_rows(m, points, "points")
    if not (math.isfinite(window) and window > 0.0):
        raise InvalidInputError(f"window: expected finite and > 0, got {window}")
    distance = as_count(distance, "distance", 0, matching.MAX_DISTANCE)
    return backends.call(
        backend,
        __name__,
        "search_by_similarity",
        reference=lambda: _search_by_similarity(
            m,
            camera,
            row,
            chosen,
            rows,
            source=m,
            window=window,
            distance=distance,
            options=options,
        ),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "k": row,
            "similarity": chosen,
            "points": rows,
            "window": float(window),
            "distance": int(distance),
            "options": options.as_dict(),
        },
        decode=lambda raw, _: _checked_ids(raw, "search_by_similarity"),
    )


def match_by_word(  # noqa: PLR0913 -- the map, its database, the two keyframes, the backend
    m: Map, db: Database, k1: int, k2: int, options: Options, *, backend: str = "python"
) -> IntArray:
    """Match two keyframes' points by word (ORB-SLAM's keyframe-to-keyframe ``SearchByBoW``).

    The keypoints of each that observe a point, matched under the vocabulary's nodes
    (:func:`slam_it.foundations.matching.search_by_bow`: ``matching.th_low``, ``loops.ratio``,
    the rotation check).

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    db : slam_it.edge.database.Database
        Its keyframe database, holding both keyframes.
    k1, k2 : int
        The two keyframes' rows.
    options : slam_it.edge.options.Options
        ``matching.th_low`` and ``loops.ratio``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (N,)
        For each keypoint of `k1`, the id of the point of `k2` it matched, -1 where none.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If either row is not a keyframe's.
    """
    first, second = maps.as_keyframe_row(m, k1, "k1"), maps.as_keyframe_row(m, k2, "k2")
    return backends.call(
        backend,
        __name__,
        "match_by_word",
        reference=lambda: _match_by_word(
            m, db, first, second, searched=_Searched(m, db, own=True), options=options
        ),
        arguments={
            "m": m.as_dict(),
            "database": db.as_dict(),
            "k1": first,
            "k2": second,
            "options": options.as_dict(),
        },
        decode=lambda raw, _: _checked_ids(raw, "match_by_word"),
    )


def group_points(
    m: Map, matched: int, linked: ArrayLike, options: Options, *, backend: str = "python"
) -> IntArray:
    """Return the points a matched keyframe's group sees (ORB-SLAM's ``FindMatchesByProjection``).

    The keyframe's best ``loops.neighbours`` links and itself; with fewer links than that, each
    link's own best, at most ``loops.neighbours`` of each, but those already in and those of
    `linked`. Their points, each once, in that order.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    matched : int
        The matched keyframe's row.
    linked : array_like of int
        The rows of the keyframe searching's links, never added by widening.
    options : slam_it.edge.options.Options
        ``loops.neighbours`` and ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64
        The points' rows.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `matched` or a row of `linked` is not a keyframe's.
    """
    row = maps.as_keyframe_row(m, matched, "matched")
    rows = np.unique(covisibility.as_points(linked))
    if np.any((rows < 0) | (rows >= m.keyframes)):
        raise InvalidInputError("linked: expected keyframe rows")
    return backends.call(
        backend,
        __name__,
        "group_points",
        reference=lambda: _group_points(m, row, set(rows.tolist()), options),
        arguments={"m": m.as_dict(), "matched": row, "linked": rows, "options": options.as_dict()},
        decode=lambda raw, _: _checked_ids(raw, "group_points"),
    )


def refine(  # noqa: PLR0913 -- the map, the two keyframes, the start, the matches, the settings
    m: Map,
    camera: Camera,
    k1: int,
    k2: int,
    *,
    similarity: ArrayLike,
    matched: ArrayLike,
    fixed_scale: bool,
    options: Options,
    backend: str = "python",
) -> Refined | None:
    """Refine the similarity between two keyframes against their matches (``OptimizeSim3``).

    Each keypoint of `k1` that observes a point and matched one carried into camera 2's frame in
    front of it gives two errors, its keypoint's in image 1 and its partner's in image 2 (none
    there where `k2` does not observe the partner), each weighed by its level's information
    (:func:`slam_it.foundations.graphs.refine_similarity`). A first round of
    ``loops.refine_iterations`` with a Huber loss at ``sqrt(loops.huber_chi2)``; the matches
    whose either chi-square exceeds ``loops.huber_chi2`` dropped; a second round with no robust
    loss, of ``loops.refine_more`` iterations if any was dropped, else
    ``loops.refine_iterations``; its inliers counted the same way.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    k1, k2 : int
        The two keyframes' rows.
    similarity : array_like, shape (4, 4)
        The starting ``S_12``, from camera 2's frame to camera 1's.
    matched : array_like of int, shape (N,)
        For each keypoint of `k1`, the id of the point it matched, -1 for none.
    fixed_scale : bool
        Refine a rigid motion.
    options : slam_it.edge.options.Options
        The ``loops`` refinement's settings.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Refined or None
        The similarity and its inliers; None when fewer than ``loops.min_kept`` matches survive
        the first round, or no match can be used.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a row is not a keyframe's, the similarity is not one, or the matches are not one per
        keypoint of `k1`.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while refining (:func:`slam_it.foundations.graphs.refine_similarity`).
    """
    first, second = maps.as_keyframe_row(m, k1, "k1"), maps.as_keyframe_row(m, k2, "k2")
    start = _similarity(similarity)
    ids = _keypoint_ids(m, first, matched, "matched")

    def decode(raw: Crossing | None, _: backends.Codes) -> Refined | None:
        if raw is None:
            return None
        found = _checked_similarity(raw["similarity"], "refined similarity")
        return Refined(similarity=found, inliers=int(raw["inliers"]))

    return backends.call(
        backend,
        __name__,
        "refine",
        reference=lambda: _refine(
            m,
            camera,
            first,
            second,
            other=m,
            similarity=start,
            matched=ids,
            fixed_scale=bool(fixed_scale),
            options=options,
        ),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "k1": first,
            "k2": second,
            "similarity": start,
            "matched": ids,
            "fixed_scale": bool(fixed_scale),
            "options": options.as_dict(),
        },
        decode=decode,
    )


def fuse_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the fusion
    m: Map,
    camera: Camera,
    k: int,
    similarity: ArrayLike,
    point_ids: ArrayLike,
    *,
    options: Options,
    backend: str = "python",
) -> tuple[Map, int]:
    """Fuse points into a keyframe through a similarity (ORB-SLAM's ``Fuse`` with ``Scw``).

    Each point still in the map that the keyframe does not observe, in view through the
    similarity (as :func:`search_by_similarity` checks it), is compared with every keypoint
    within ``loops.fuse_window`` times its predicted level's scale, on that level or the one
    below; the nearest within ``matching.th_low`` takes it: a free keypoint observes it, and a
    keypoint observing another point has that point replaced by it
    (:func:`slam_it.edge.map.replace_point`: the loop's point is kept, as ORB-SLAM's loop
    fusion keeps it).

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    k : int
        The keyframe's row.
    similarity : array_like, shape (4, 4)
        Its ``S_cw``.
    point_ids : array_like of int
        The points' ids, in the order fused; one no longer in the map is passed over.
    options : slam_it.edge.options.Options
        ``loops.fuse_window``, ``matching.th_low`` and ``tracking.view_cos_limit``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    tuple of slam_it.edge.map.Map and int
        The map, and the points fused.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `k` is not a keyframe row, the similarity is not one, or an id is negative.
    """
    row = maps.as_keyframe_row(m, k)
    chosen = _similarity(similarity)
    ids = covisibility.as_points(point_ids)
    if np.any(ids < 0):
        raise InvalidInputError("point_ids: expected point ids")

    def decode(raw: Crossing, codes: backends.Codes) -> tuple[Map, int]:
        return maps.decode(raw["map"], codes), int(raw["fused"])

    return backends.call(
        backend,
        __name__,
        "fuse_by_similarity",
        reference=lambda: _fuse_by_similarity(m, camera, row, chosen, ids, options=options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "k": row,
            "similarity": chosen,
            "point_ids": ids,
            "options": options.as_dict(),
        },
        decode=decode,
    )


def global_bundle_adjust(
    m: Map,
    camera: Camera,
    options: Options,
    *,
    stop: Callable[[], bool] | None = None,
    backend: str = "python",
) -> tuple[Map, graphs.Summary]:
    """Bundle-adjust every keyframe and point (``GlobalBundleAdjustemnt``, as loop closing runs it).

    Every observation, weighed by its level's information, with its right column where it has
    one; no robust loss; the map's first keyframe (id 0) held fixed; ``loops.global_iterations``
    (:func:`slam_it.foundations.graphs.bundle_adjust`). Every point is refreshed after.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map, with its first keyframe.
    camera : slam_it.edge.frames.Camera
        The camera.
    options : slam_it.edge.options.Options
        ``loops.global_iterations``.
    stop : callable, optional
        Asked before each iteration whether to stop (a global BA in a thread of its own,
        superseded: ORB-SLAM's ``mbStopGBA``); the Python reference's alone.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    tuple of slam_it.edge.map.Map and slam_it.foundations.graphs.Summary
        The map adjusted, and the solve's summary.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the map does not hold its first keyframe.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while solving (:func:`slam_it.foundations.graphs.bundle_adjust`).
    """
    if not np.any(m.kf_id == 0):
        raise InvalidInputError("m: expected the map's first keyframe (id 0)")
    if stop is not None and backend != "python":
        raise InvalidInputError("stop: a compiled backend's core takes its own stop flag")

    def decode(raw: Crossing, codes: backends.Codes) -> tuple[Map, graphs.Summary]:
        return maps.decode(raw["map"], codes), graphs.summary_from_backend(raw["summary"], codes)

    return backends.call(
        backend,
        __name__,
        "global_bundle_adjust",
        reference=lambda: _global_bundle_adjust(m, camera, options, stop),
        arguments={"m": m.as_dict(), "camera": camera.as_dict(), "options": options.as_dict()},
        decode=decode,
    )


def detect(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the state
    m: Map,
    db: Database,
    camera: Camera,
    keyframe: int,
    pending: Pending | None,
    *,
    mode: str,
    options: Options,
    backend: str = "python",
) -> Detection:
    """Look for a loop at a newly mapped keyframe (ORB-SLAM3's ``NewDetectCommonRegions``).

    The module docstring's detection: the loop being confirmed carried to the keyframe, and,
    when it is not confirmed here, the database's candidates tried.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map, holding the keyframe.
    db : slam_it.edge.database.Database
        Its keyframe database, holding the keyframe.
    camera : slam_it.edge.frames.Camera
        The camera.
    keyframe : int
        The new keyframe's id.
    pending : Pending or None
        The loop being confirmed, from the keyframes before; None if none is.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode: the similarity is rigid but in monocular.
    options : slam_it.edge.options.Options
        The ``loops`` settings, and those the searches read.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Detection
        The loop being confirmed after the keyframe, the loop closed on it, and how it went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the keyframe is not in the map, the mode is unknown, or the loop being confirmed
        holds a similarity that is not one.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while refining a candidate's similarity
        (:func:`slam_it.foundations.graphs.refine_similarity`).
    """
    check_mode(mode)
    maps.keyframe_row(m, keyframe)
    if pending is not None:
        _similarity(pending.similarity, "pending.similarity")
    return backends.call(
        backend,
        __name__,
        "detect",
        reference=lambda: _detect(m, db, camera, keyframe, pending, mode=mode, options=options),
        arguments={
            "m": m.as_dict(),
            "database": db.as_dict(),
            "camera": camera.as_dict(),
            "keyframe": int(keyframe),
            "pending": None if pending is None else _pending_as_dict(pending),
            "mode": mode,
            "options": options.as_dict(),
        },
        decode=_decode_detection,
    )


def correct(  # noqa: PLR0913 -- the map, its camera, the loop, the settings, the backend
    m: Map, camera: Camera, loop: Loop, *, mode: str, options: Options, backend: str = "python"
) -> Corrected:
    """Correct a loop closed at a keyframe (ORB-SLAM3's ``CorrectLoop``; the module docstring).

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map, holding both keyframes.
    camera : slam_it.edge.frames.Camera
        The camera.
    loop : Loop
        The loop.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode: the essential graph is Sim(3) but in stereo and RGB-D.
    options : slam_it.edge.options.Options
        The ``loops`` settings, and those the fusion reads.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Corrected
        The map corrected, and how it went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If either keyframe is not in the map, the mode is unknown, the similarity is not one, or
        the matched points are not one per keypoint of the current keyframe.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while optimising the essential graph or in the global BA.
    """
    check_mode(mode)
    current = maps.keyframe_row(m, loop.current)
    maps.keyframe_row(m, loop.matched)
    _similarity(loop.similarity, "loop.similarity")
    _keypoint_ids(m, current, loop.matched_points, "loop.matched_points")
    _ids(loop.points, "loop.points")
    return backends.call(
        backend,
        __name__,
        "correct",
        reference=lambda: _correct(m, camera, loop, mode=mode, options=options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "loop": _loop_as_dict(loop),
            "mode": mode,
            "options": options.as_dict(),
        },
        decode=_decode_corrected,
    )


def detect_merge(  # noqa: PLR0913 -- the map, its database and camera, the keyframe, the state
    m: Map,
    db: Database,
    camera: Camera,
    keyframe: int,
    *,
    others: Sequence[database.Indexed],
    pending: Merging | None,
    mode: str,
    options: Options,
    backend: str = "python",
) -> MergeDetection:
    """Look for another map at a newly mapped keyframe (ORB-SLAM3's merge detection).

    ``NewDetectCommonRegions``' merge branch, beside loop detection and by its stages (the
    module docstring's): nothing while the active map holds fewer than ``loops.min_keyframes``;
    the merge being confirmed carried to the keyframe, in its matched map; and, when it is not
    confirmed here, the other maps' candidates tried (:func:`slam_it.edge.database
    .merge_candidates`), each against its own map, which holds none of the keyframe's links.
    Unlike ORB-SLAM3's source, a confirmation resets the merge's misses as it resets a loop's,
    so two misses in a row drop it. A monocular merge's similarity carries the two maps' scale
    ratio, unbounded (ORB-SLAM3 bounds it for inertial maps alone). :func:`merge_first` gives
    a merge priority over a loop found on the same keyframe.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The active map, holding the keyframe.
    db : slam_it.edge.database.Database
        Its keyframe database, holding the keyframe.
    camera : slam_it.edge.frames.Camera
        The camera.
    keyframe : int
        The new keyframe's id.
    others : sequence of slam_it.edge.database.Indexed
        The other maps, each with its database and number.
    pending : Merging or None
        The merge being confirmed, from the keyframes before; None if none is.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode: the similarity is rigid but in monocular.
    options : slam_it.edge.options.Options
        The ``loops`` settings, and those the searches read.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    MergeDetection
        The merge being confirmed after the keyframe, the merge found on it, and how it went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the keyframe is not in the map, the mode is unknown, two maps share a number, the
        merge being confirmed names a map not among `others` or holds a similarity that is not
        one.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while refining a candidate's similarity
        (:func:`slam_it.foundations.graphs.refine_similarity`).
    """
    check_mode(mode)
    maps.keyframe_row(m, keyframe)
    by_number = {other.number: other for other in others}
    if len(by_number) != len(others):
        raise InvalidInputError("others: expected each map once, by its number")
    if pending is not None:
        if pending.map not in by_number:
            raise InvalidInputError(f"pending.map: map {pending.map} is not among the others")
        _similarity(pending.pending.similarity, "pending.similarity")
    return backends.call(
        backend,
        __name__,
        "detect_merge",
        reference=lambda: _detect_merge(
            m, db, camera, keyframe, others=by_number, pending=pending, mode=mode, options=options
        ),
        arguments={
            "m": m.as_dict(),
            "database": db.as_dict(),
            "camera": camera.as_dict(),
            "keyframe": int(keyframe),
            "others": [other.as_dict() for other in others],
            "pending": _merging_as_dict(pending),
            "mode": mode,
            "options": options.as_dict(),
        },
        decode=_decode_merge_detection,
    )


def merge_first(loop: Detection, merging: MergeDetection, *, backend: str = "python") -> Detection:
    """Give a merge found on a keyframe priority over a loop closed on it (ORB-SLAM3's ``Run``).

    Where both were found on one keyframe, the merge is made and the loop forgotten, as
    ORB-SLAM3 resets its loop's state after a merge, its outcome :attr:`Outcome.MERGED`; a loop
    still being confirmed is kept.

    Parameters
    ----------
    loop : Detection
        Loop detection's result on the keyframe.
    merging : MergeDetection
        Merge detection's on the same keyframe.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Detection
        `loop`, its closed loop and its state dropped, its outcome ``merged``, if a merge was
        found too.
    """
    return backends.call(
        backend,
        __name__,
        "merge_first",
        reference=lambda: _merge_first(loop, merging),
        arguments={
            "loop": _detection_as_dict(loop),
            "merging": _merge_detection_as_dict(merging),
        },
        decode=_decode_detection,
    )


def merge(  # noqa: PLR0913 -- the two maps, the camera, the merge, the settings
    active: Map,
    matched: Map,
    camera: Camera,
    found: Merge,
    *,
    mode: str,
    options: Options,
    backend: str = "python",
) -> Merged:
    """Merge the active map into the one it matched (ORB-SLAM3's ``MergeLocal``, as its paper).

    The online-SLAM design's section 14, item 4:

    - the matched map, the older, survives with its world, scale and ids; the active map is
      carried into its world by the merge's similarity (each keyframe as the current one sees
      it, each point with it) and taken in after it, every id of its, live or archived, moved
      on by the matched map's next id (:class:`Merged`);
    - the active side's spanning tree is re-rooted at the current keyframe and hung under the
      matched one, so the merged map's root is the matched map's;
    - the welding windows: each of the two keyframes and its best links
      (``loops.weld_neighbours``, widened by ``loops.weld_widen`` up to ``loops.weld_rounds``
      times);
    - the matched window's points fused into the active window's keyframes (``SearchAndFuse``:
      ``loops.fuse_window``, ``matching.th_low``), each replacing the point a keypoint had;
    - the welding BA (the paper's: both windows optimised, the matched map's keyframes outside
      them that see their points held; ORB-SLAM3's source holds the matched window instead);
    - the essential graph over the whole merged map, both windows held (:func:`essential_graph`,
      the paper's; ORB-SLAM3's source covers only the rest of the active map, in stereo and
      RGB-D, and moves a monocular one by one similarity), the active side's nodes in its own
      units, so its edges hold;
    - global BA after (``loops.global_ba``), as after a loop; ORB-SLAM3's source runs it only
      where one was interrupted.

    Parameters
    ----------
    active : slam_it.edge.map.Map
        The active map, holding the merge's current keyframe.
    matched : slam_it.edge.map.Map
        The map it matched, holding the merge's matched keyframe.
    camera : slam_it.edge.frames.Camera
        The camera.
    found : Merge
        The merge (:func:`detect_merge`).
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode: the similarity's scale is used but in stereo and RGB-D.
    options : slam_it.edge.options.Options
        The ``loops`` and ``adjustment`` settings, and those the fusion reads.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Merged
        The merged map, the two offsets that carried the active map's ids, and how it went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If either keyframe is not in its map, the mode is unknown, the similarity is not one,
        or the maps' pyramids differ.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while solving the welding BA, the essential graph or global BA
        (:mod:`slam_it.foundations.graphs`).
    """
    check_mode(mode)
    region = found.loop
    maps.keyframe_row(active, region.current)
    maps.keyframe_row(matched, region.matched)
    _similarity(region.similarity, "found.loop.similarity")
    if (active.levels, active.scale_factor) != (matched.levels, matched.scale_factor):
        raise InvalidInputError("merge: the two maps' pyramids differ")
    return backends.call(
        backend,
        __name__,
        "merge",
        reference=lambda: _merge(active, matched, camera, found, mode=mode, options=options),
        arguments={
            "active": active.as_dict(),
            "matched": matched.as_dict(),
            "camera": camera.as_dict(),
            "found": _merge_as_dict(found),
            "mode": mode,
            "options": options.as_dict(),
        },
        decode=_decode_merged,
    )
