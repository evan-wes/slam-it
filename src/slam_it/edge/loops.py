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
  keyframe's links, at most ``loops.neighbours`` of each, as ORB-SLAM's code means to; ORB-SLAM
  adds them all;
- a group is the candidate and its best links, in their order; ORB-SLAM moves the first link
  to the end;
- a point carried behind a keyframe is never matched or fused; the moved keyframes' points
  are moved with the first of them that sees them, the new keyframe first, then its links,
  most shared first, where ORB-SLAM's order is its keyframes' addresses in memory.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.edge import covisibility, database, residuals
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.frames import Camera
from slam_it.edge.map import Map
from slam_it.edge.options import Options, check_mode
from slam_it.foundations import camera as cameras
from slam_it.foundations import graphs, matching, ransac, sim3
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


def _search_by_similarity(  # noqa: PLR0913 -- the map, its camera, the keyframe, the search
    m: Map,
    camera: Camera,
    k: int,
    similarity: Array,
    points: IntArray,
    *,
    window: float,
    distance: int,
    options: Options,
) -> IntArray:
    """Match points to a keyframe's keypoints through a similarity (reference)."""
    start = int(m.kf_start[k])
    out = np.full(int(m.kf_start[k + 1]) - start, -1, dtype=np.int64)
    seen = _in_view(m, camera, similarity, points, options)
    for p, pixel, level in zip(seen.rows, seen.pixels, seen.levels.tolist(), strict=True):
        near = _candidates_near(m, k, pixel, window * m.scale_factor**level, level)
        near = near[out[near - start] < 0]
        if len(near) == 0:
            continue
        hamming = matching.distances(m.pt_descriptor[p : p + 1], m.kp_descriptor[near])[0]
        best = int(np.argmin(hamming))
        if hamming[best] <= distance:
            out[near[best] - start] = m.pt_id[p]
    return out


def _match_by_word(m: Map, db: Database, k1: int, k2: int, options: Options) -> IntArray:
    """Match two keyframes' points by word (reference)."""
    rows1, rows2 = (np.arange(m.kf_start[k], m.kf_start[k + 1], dtype=np.int64) for k in (k1, k2))
    nodes1, nodes2 = (db.nodes(int(m.kf_id[k])) for k in (k1, k2))
    own1, own2 = rows1[m.kp_point[rows1] >= 0], rows2[m.kp_point[rows2] >= 0]
    found = matching.search_by_bow(
        m.kp_descriptor[own1],
        nodes1[own1 - rows1[0]] if len(rows1) else np.zeros(0, dtype=np.int64),
        m.kp_descriptor[own2],
        nodes2[own2 - rows2[0]] if len(rows2) else np.zeros(0, dtype=np.int64),
        m.kp_angle[own1],
        m.kp_angle[own2],
        matching.Options(
            max_distance=options.matching.th_low, ratio=options.loops.ratio, check_orientation=True
        ),
    )
    out = np.full(len(rows1), -1, dtype=np.int64)
    out[own1[found.pairs[:, 0]] - (rows1[0] if len(rows1) else 0)] = m.kp_point[
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
    similarity: Array,
    matched: IntArray,
    fixed_scale: bool,
    options: Options,
) -> Refined | None:
    """Refine the similarity between two keyframes against their matches (reference)."""
    settings = options.loops
    rows1 = np.arange(m.kf_start[k1], m.kf_start[k1 + 1], dtype=np.int64)
    use = (matched >= 0) & (m.kp_point[rows1] >= 0)
    own = m.point_rows(m.kp_point[rows1[use]])
    partner = m.point_rows(matched[use])
    keep = (own >= 0) & (partner >= 0)
    rows, own, partner = rows1[use][keep], own[keep], partner[keep]
    points1 = maps.to_camera(m.kf_pose[k1], m.pt_xyz[own])
    points2 = maps.to_camera(m.kf_pose[k2], m.pt_xyz[partner])
    front = (points1[:, 2] > cameras.MIN_DEPTH) & (points2[:, 2] > cameras.MIN_DEPTH)
    rows, partner, points1, points2 = rows[front], partner[front], points1[front], points2[front]
    if len(rows) == 0:
        return None
    rows2 = np.arange(m.kf_start[k2], m.kf_start[k2 + 1], dtype=np.int64)
    pixels2 = np.full((len(rows), 2), np.nan)
    level2 = np.zeros(len(rows), dtype=np.int64)
    for i, point_id in enumerate(m.pt_id[partner].tolist()):
        at = rows2[m.kp_point[rows2] == point_id]
        if len(at):
            pixels2[i], level2[i] = m.kp_xy[at[0]], m.kp_level[at[0]]
    information = np.column_stack(
        [1.0 / residuals.sigma2(m, m.kp_level[rows]), 1.0 / residuals.sigma2(m, level2)]
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
    options: Options,
) -> tuple[IntArray, IntArray]:
    """Search a matched keyframe's group's points in a keyframe through a similarity.

    ORB-SLAM's ``FindMatchesByProjection``: :func:`group_points`, then
    :func:`search_by_similarity` with ``loops.confirm_window`` and ``loops.confirm_distance``.
    Return the group's points' ids and each of the keyframe's keypoints' match.
    """
    points = _group_points(m, matched, _linked(m, k, options), options)
    found = _search_by_similarity(
        m, camera, k, similarity, points,
        window=options.loops.confirm_window, distance=options.loops.confirm_distance,
        options=options,
    )  # fmt: skip
    return m.pt_id[points], found


def _matched(found: IntArray) -> int:
    """Return how many keypoints a search matched."""
    return int(np.count_nonzero(found >= 0))


def _carry(  # noqa: PLR0913 -- the map, its camera, the keyframe, the loop, the settings
    m: Map, camera: Camera, k: int, pending: Pending, *, fixed_scale: bool, options: Options
) -> tuple[Pending | None, ConfirmationSummary]:
    """Try to confirm the loop being confirmed on a later keyframe (module docstring, step 1).

    Return the loop as it stands after the keyframe, None once dropped, and how it went.
    """
    settings = options.loops
    current, matched = m.kf_pose[k], maps.keyframe_row(m, pending.matched)
    last = maps.keyframe_row(m, pending.last)
    carried = _relative(current, m.kf_pose[last]) @ pending.similarity
    points, found = _confirm_by_projection(m, camera, k, matched, carried, options=options)
    summary = ConfirmationSummary(matched=pending.matched, projected=_matched(found))
    confirmed: tuple[Array, IntArray, IntArray] | None = None
    if summary.projected >= settings.confirm_matches:
        start = carried @ sim3.inverse(m.kf_pose[matched])
        refined = _refine(
            m, camera, k, matched, similarity=start, matched=found, fixed_scale=fixed_scale,
            options=options,
        )  # fmt: skip
        summary = replace(summary, refined=None if refined is None else refined.inliers)
        if refined is not None and refined.inliers > settings.temporal_refined:
            placed = refined.similarity @ m.kf_pose[matched]
            points, again = _confirm_by_projection(m, camera, k, matched, placed, options=options)
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


def _merged_by_word(
    m: Map, db: Database, k: int, group: list[int], options: Options
) -> tuple[IntArray, IntArray, int]:
    """Match a keyframe by word to each of a group's keyframes, and merge the matches.

    Each keypoint takes its first match in the group's order whose point no earlier keypoint
    took. Return each keypoint's point id and the row of the keyframe it was matched in (-1
    where none), and the row of the group's keyframe that matched most (ties: the first).
    """
    size = int(m.kf_start[k + 1] - m.kf_start[k])
    merged = np.full(size, -1, dtype=np.int64)
    source = np.full(size, -1, dtype=np.int64)
    used: set[int] = set()
    best, most = group[0], -1
    for g in group:
        found = _match_by_word(m, db, k, g, options)
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
    source: IntArray,
    fixed_scale: bool,
    options: Options,
) -> tuple[Array | None, int | None]:
    """Find the similarity from the best keyframe's frame to the keyframe's by RANSAC.

    Each merged match gives the keyframe's own point in its frame and the matched point in the
    best keyframe's, both in front, each with its keypoint's level variance (the matched
    point's in the keyframe it was matched in). Return ``S_kb`` and its inliers, or None and
    the inliers it found (None if it did not run) when it falls short of ``loops.min_inliers``.
    """
    settings = options.loops
    rows = m.kf_start[k] + np.flatnonzero(merged >= 0)
    own = m.point_rows(m.kp_point[rows])
    partner = m.point_rows(merged[merged >= 0])
    points1 = maps.to_camera(m.kf_pose[k], m.pt_xyz[own])
    points2 = maps.to_camera(m.kf_pose[best], m.pt_xyz[partner])
    level2 = np.zeros(len(rows), dtype=np.int64)
    for i, (point_id, g) in enumerate(
        zip(merged[merged >= 0].tolist(), source[merged >= 0].tolist(), strict=True)
    ):
        at = np.flatnonzero(m.kp_point[m.keypoint_rows(g)] == point_id)
        level2[i] = m.kp_level[m.kf_start[g] + at[0]]
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
            residuals.sigma2(m, level2[front]), run, model=camera.model,
            intrinsics=camera.intrinsics, fixed_scale=fixed_scale,
        )  # fmt: skip
    except DegenerateError:
        return None, 0
    inliers = found.summary.inlier_count
    if inliers < settings.min_inliers:
        return None, inliers
    return found.state.model, inliers


def _confirmations(  # noqa: PLR0913 -- the map, its camera, the keyframe, the loop
    m: Map, camera: Camera, k: int, best: int, similarity: Array, *, options: Options
) -> int:
    """Count the keyframe's best links that confirm the loop, at most ``loops.confirmations``."""
    count = 0
    for j in _best_links(m, k, options):
        if count == options.loops.confirmations:
            break
        carried = _relative(m.kf_pose[j], m.kf_pose[k]) @ similarity
        _, found = _confirm_by_projection(m, camera, j, best, carried, options=options)
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
    summary: CandidateSummary,
    fixed_scale: bool,
    options: Options,
) -> tuple[tuple[Array, IntArray, IntArray] | None, CandidateSummary]:
    """Place a candidate's loop: search, refine, search again (module docstring, step 2).

    The best keyframe's group's points searched through ``S_kb`` (`model`), the similarity
    refined against those matches, and the points searched through the refined one. Return the
    keyframe's ``S_cw``, the group's points' rows and the second search's matches, or None where
    a test failed; and the summary with each count reached.
    """
    settings = options.loops
    points = _keyframe_points(m, [*_best_links(m, best, options), best])
    coarse = _search_by_similarity(
        m, camera, k, model @ m.kf_pose[best], points,
        window=settings.coarse_window, distance=settings.coarse_distance, options=options,
    )  # fmt: skip
    summary = replace(summary, projected=_matched(coarse))
    if _matched(coarse) < settings.coarse_matches:
        return None, summary
    refined = _refine(
        m, camera, k, best, similarity=model, matched=coarse, fixed_scale=fixed_scale,
        options=options,
    )  # fmt: skip
    summary = replace(summary, refined=None if refined is None else refined.inliers)
    if refined is None or refined.inliers < settings.min_refined:
        return None, summary
    placed = refined.similarity @ m.kf_pose[best]
    fine = _search_by_similarity(
        m, camera, k, placed, points,
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
    linked: set[int],
    fixed_scale: bool,
    options: Options,
) -> tuple[CandidateSummary, _Found | None]:
    """Try one of the database's candidates (module docstring, step 2)."""
    settings = options.loops
    keyframe, score = candidate
    c = maps.keyframe_row(m, keyframe)
    group = [c, *_best_links(m, c, options)]
    summary = CandidateSummary(keyframe=keyframe, score=score)
    if linked & set(group):
        return replace(summary, near=True), None
    merged, source, best = _merged_by_word(m, db, k, group, options)
    summary = replace(summary, matches=_matched(merged), best=int(m.kf_id[best]))
    if _matched(merged) < settings.min_matches:
        return summary, None
    model, inliers = _similarity_by_ransac(
        m, camera, k, best, merged, source=source, fixed_scale=fixed_scale, options=options
    )
    summary = replace(summary, inliers=inliers)
    if model is None:
        return summary, None
    placed, summary = _place(
        m, camera, k, best, model, summary=summary, fixed_scale=fixed_scale, options=options
    )
    if placed is None:
        return summary, None
    similarity, points, fine = placed
    confirmations = _confirmations(m, camera, k, best, similarity, options=options)
    found = _Found(
        best=best,
        similarity=similarity,
        points=m.pt_id[points],
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
    linked = _linked(m, k, options)
    tried: list[CandidateSummary] = []
    best: _Found | None = None
    for keyframe, score in zip(
        candidates.keyframes.tolist(), candidates.scores.tolist(), strict=True
    ):
        summary, found = _try_candidate(
            m, db, camera, k, (keyframe, score), linked=linked, fixed_scale=fixed_scale,
            options=options,
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
            m, camera, k, pending, fixed_scale=fixed_scale, options=options
        )
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
        for point_id in m.kp_point[m.keypoint_rows(r)].tolist():
            if point_id < 0 or point_id in moved_by:
                continue
            p = int(m.point_rows(np.array([point_id], dtype=np.int64))[0])
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
    fixed_scale: bool,
    options: Options,
) -> tuple[list[Array], int, graphs.Summary]:
    """Optimise the essential graph from the loop's start (ORB-SLAM's ``OptimizeEssentialGraph``).

    A node per keyframe, starting from `start`; an edge for each of: the loop's new links
    sharing ``loops.essential_points`` points, and the current keyframe's to the matched one
    whatever it shares, each measured between the starting similarities; and, measured between
    the poses before the correction, each keyframe's to its parent, its loop edges to older
    keyframes, and its links sharing ``loops.essential_points`` points to older keyframes but its
    parent and children. Unit information, the matched keyframe held fixed,
    ``loops.essential_iterations`` of Levenberg-Marquardt from ``loops.essential_damping``
    (:func:`slam_it.foundations.graphs.optimize_pose_graph`), on Sim(3) but in stereo and RGB-D.

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
    fixed_scale : bool
        Optimise SE(3), not Sim(3).
    options : slam_it.edge.options.Options
        The ``loops`` settings.

    Returns
    -------
    tuple
        Each keyframe's optimised ``S_iw``, the edges, and the solve's summary.
    """
    graph = _essential_edges(m, start, before, loop_links, ends, options=options)
    solved = graphs.optimize_pose_graph(
        np.array([sim3.inverse(s) for s in start]),
        np.array(graph.edges, dtype=np.int64).reshape(-1, 2),
        np.array(graph.relatives).reshape(-1, 4, 4),
        [ends[1]],
        similarity=not fixed_scale,
        options=graphs.Options(
            max_iterations=options.loops.essential_iterations,
            initial_damping=options.loops.essential_damping,
        ),
    )
    return [sim3.inverse(pose) for pose in solved.poses], len(graph.edges), solved.summary


def _global_bundle_adjust(m: Map, camera: Camera, options: Options) -> tuple[Map, graphs.Summary]:
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
    moved_ids = [int(m.kf_id[r]) for r in moved_rows]
    before = list(m.kf_pose.copy())
    corrected = {k: loop.similarity}
    to_world = sim3.inverse(m.kf_pose[k])
    for r in links:
        corrected[r] = m.kf_pose[r] @ to_world @ loop.similarity
    previous = {r: _linked(m, r, options) for r in moved_rows}
    m, moved_by, moved = _move_keyframes(m, moved_rows, corrected)
    m, replaced = _replace_matched(m, k, loop.matched_points)
    fused = 0
    for keyframe in moved_ids:
        r = maps.keyframe_row(m, keyframe)
        m, count = _fuse_by_similarity(
            m, camera, r, corrected[moved_rows[moved_ids.index(keyframe)]], loop.points,
            options=options,
        )  # fmt: skip
        fused += count
    moved_now = [maps.keyframe_row(m, keyframe) for keyframe in moved_ids]
    loop_links = {
        r: _linked(m, r, options) - previous[old] - set(moved_now)
        for r, old in zip(moved_now, moved_rows, strict=True)
    }
    start = list(m.kf_pose.copy())
    for r, old in zip(moved_now, moved_rows, strict=True):
        start[r] = corrected[old]
    # The keyframes are the same rows before and after: nothing here adds or removes one.
    optimised, edges, essential = essential_graph(
        m, start, before, loop_links, (maps.keyframe_row(m, loop.current), matched),
        fixed_scale=fixed_scale, options=options,
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


def _move_by_graph(
    m: Map, start: list[Array], optimised: list[Array], moved_by: dict[int, int]
) -> Map:
    """Set each keyframe to its optimised pose, and move each point with its keyframe.

    A point moved by the loop goes with the keyframe that moved it; any other with its reference
    keyframe (ORB-SLAM's ``mnCorrectedReference``, else ``GetReferenceKeyFrame``).
    """
    xyz = m.pt_xyz.copy()
    for p, point_id in enumerate(m.pt_id.tolist()):
        r = moved_by.get(point_id)
        if r is None:
            r = int(m.keyframe_rows(m.pt_reference_kf[p : p + 1])[0])
            if r < 0:
                continue
        carry = sim3.inverse(optimised[r]) @ start[r]
        xyz[p] = carry[:3, :3] @ m.pt_xyz[p] + carry[:3, 3]
    poses = np.array([rigid_of(s) for s in optimised]).reshape(-1, 4, 4)
    out = replace(m, kf_pose=poses, pt_xyz=xyz)
    return maps.refresh_points(out, np.arange(out.points))


# --- the backends -------------------------------------------------------------------------

Crossing = dict[str, Any]
"""A value as it crosses the bindings: a dict of arrays, numbers, and dicts of them."""


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.loops`` submodule provides.

    The map, its database, the camera and the settings cross as dicts (their ``as_dict``); a
    similarity as its 4x4 matrix; a :class:`Pending` and a :class:`Loop` as their
    :meth:`~Pending.as_dict`; the mode by name. Results cross back as dicts: a detection's
    ``pending`` and ``loop`` (each a dict or None) and ``summary``, its outcome a code in
    ``codes()["loop_outcome"]``; a correction's ``map`` and ``summary``, each solve's
    termination a code in ``codes()["graphs_termination"]``.
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
    """Check a vector of ids: integers, at least -1."""
    out = covisibility.as_points(values)
    if np.any(out < -1):
        raise InvalidInputError(f"{name}: expected ids of at least -1")
    return out


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
    summary = raw["summary"]
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
    return Detection(
        pending=_decode_pending(raw["pending"]),
        loop=_decode_loop(raw["loop"]),
        summary=DetectionSummary(
            outcome=outcome,
            confirmation=confirmation,
            sharing=int(summary["sharing"]),
            scored=int(summary["scored"]),
            candidates=candidates,
            confirmations=int(summary["confirmations"]),
        ),
    )


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
        If `k` is not a keyframe row, the similarity is not one, or a point row is out of range.
    """
    row = maps.as_keyframe_row(m, k)
    chosen = _similarity(similarity)
    rows = maps.as_point_rows(m, points, "points")
    return backends.call(
        backend,
        __name__,
        "search_by_similarity",
        reference=lambda: _search_by_similarity(
            m, camera, row, chosen, rows, window=window, distance=distance, options=options
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
        reference=lambda: _match_by_word(m, db, first, second, options),
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
    m: Map, camera: Camera, options: Options, *, backend: str = "python"
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
    """
    if not np.any(m.kf_id == 0):
        raise InvalidInputError("m: expected the map's first keyframe (id 0)")

    def decode(raw: Crossing, codes: backends.Codes) -> tuple[Map, graphs.Summary]:
        return maps.decode(raw["map"], codes), graphs.summary_from_backend(raw["summary"], codes)

    return backends.call(
        backend,
        __name__,
        "global_bundle_adjust",
        reference=lambda: _global_bundle_adjust(m, camera, options),
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
