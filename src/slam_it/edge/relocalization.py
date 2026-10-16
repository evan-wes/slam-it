"""Relocalisation: place a frame in the map through the keyframes that look like it.

ORB-SLAM's ``Tracking::Relocalization`` (Mur-Artal and Tardós, T-RO 2017; ORB-SLAM3's, Campos
et al., T-RO 2021), against the map being tracked:

1. **The candidates:** the frame's features quantised and its word vector made
   (:mod:`slam_it.foundations.vocabulary`), and the keyframe database's candidates found
   (:func:`slam_it.edge.database.candidates`), best first.
2. **For each candidate in turn,** its keypoints that observe a point are matched to the frame's
   by word (:func:`slam_it.foundations.matching.search_by_bow`: ``th_low``,
   ``relocalize_ratio``, the rotation check; ``SearchByBoW``); with fewer than
   ``reloc_min_matches``, the next candidate.
3. **PnP RANSAC** on those matches (:func:`slam_it.foundations.pnp.pnp_ransac`,
   ``pnp_threshold`` pixels, ``seed``), set as ORB-SLAM sets MLPnP's
   (``SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991)``): the pose needs
   ``n = max(reloc_min_inliers, floor(reloc_inlier_ratio * matches))`` inliers, and the
   samples drawn are those that find such a pose with ``reloc_confidence``,
   ``ceil(log(1 - p) / log(1 - e^3))`` for ``e = max(reloc_inlier_ratio, n / matches)``, at
   most ``reloc_ransac_iterations`` (35 at the defaults). Fewer inliers: the next candidate.
4. **The pose optimised** (:func:`slam_it.edge.tracking.optimize_pose`); fewer than
   ``reloc_min_inliers`` inliers: the next candidate. Fewer than ``recovery_tracked`` (50): the
   candidate's other points searched by projection (:func:`search_keyframe`, a window of
   ``reloc_window`` pixels at the predicted level and ``th_high``), and if that brings 50, the
   pose optimised again; then with more than ``reloc_narrow_inliers`` but fewer than 50, the
   search repeated narrower (``reloc_narrow_window``, ``reloc_narrow_distance``) and, if that
   brings 50, the pose optimised once more. 50 inliers place the frame, and end the search.

**Unlike ORB-SLAM** (decided with evan-wes on 2026-10-15): the candidates are tried one at a
time, best first, each RANSAC run to its end, its best pose kept (LO-MSAC, stopping once
confident); ORB-SLAM runs five samples of each in turn, in the order its index found them, and
takes the first pose with enough inliers. ORB-SLAM counts its samples with ``e^3`` though MLPnP
draws 6 points; P3P draws 3, so the count is right for it. The RANSAC's threshold is
``pnp_threshold`` at every level, where MLPnP's is 5.991 times the level's variance. The search
by projection skips a point behind the camera, which ORB-SLAM's projects regardless.

Each candidate tried reports how far it got (:class:`CandidateSummary`), and the frame how the
search went (:class:`RelocalizationSummary`), which a run records.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from slam_it import backends
from slam_it.edge import database, frames, tracking
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.frames import Camera, Frame, Tracked
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations import matching, p3p, ransac, vocabulary
from slam_it.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


@dataclass(frozen=True)
class CandidateSummary:
    """How far one candidate got, as a run records it; each count None where it stopped first.

    Attributes
    ----------
    keyframe : int
        The candidate's id in the map.
    score : float
        Its group's accumulated score.
    matches : int
        The frame's keypoints matched to its points by word, with a bearing in front.
    iterations : int or None
        RANSAC's samples, if it ran and found a pose.
    termination : slam_it.foundations.ransac.Termination or None
        Why RANSAC stopped, if it found a pose.
    degenerate : bool
        Whether RANSAC ran and found no pose.
    inliers : int or None
        RANSAC's inliers.
    refined : int or None
        The inliers the first optimisation kept.
    found : int or None
        The inliers after the searches by projection: the last optimisation's.
    """

    keyframe: int
    score: float
    matches: int
    iterations: int | None = None
    termination: ransac.Termination | None = None
    degenerate: bool = False
    inliers: int | None = None
    refined: int | None = None
    found: int | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``keyframe``, ``score``, ``matches``, ``iterations``, ``termination`` (by name;
            ``"degenerate"`` when RANSAC found no pose), ``inliers``, ``refined`` and
            ``found``.
        """
        termination = None if self.termination is None else self.termination.value
        return {
            "keyframe": self.keyframe,
            "score": self.score,
            "matches": self.matches,
            "iterations": self.iterations,
            "termination": "degenerate" if self.degenerate else termination,
            "inliers": self.inliers,
            "refined": self.refined,
            "found": self.found,
        }


@dataclass(frozen=True)
class RelocalizationSummary:
    """How relocalising a frame went, as a run records it.

    Attributes
    ----------
    words : int
        The frame's words with a weight (its vector's length).
    sharing : int
        The keyframes that shared a word with it.
    scored : int
        Those scored.
    candidates : tuple of CandidateSummary
        The candidates tried, in order; the last placed the frame if any did.
    placed : bool
        Whether the frame was placed.
    """

    words: int
    sharing: int = 0
    scored: int = 0
    candidates: tuple[CandidateSummary, ...] = ()
    placed: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``words``, ``sharing``, ``scored``, ``candidates`` (a list of
            :meth:`CandidateSummary.as_dict`) and ``placed``.
        """
        return {
            "words": self.words,
            "sharing": self.sharing,
            "scored": self.scored,
            "candidates": [candidate.as_dict() for candidate in self.candidates],
            "placed": self.placed,
        }


@dataclass(frozen=True)
class Relocalized:
    """Relocalisation's result: the frame, if it was placed, and how the search went.

    Attributes
    ----------
    tracked : slam_it.edge.frames.Tracked or None
        The frame placed, with its inliers; None if no candidate placed it.
    summary : RelocalizationSummary
        How it went.
    matched : ndarray of int64, shape (N,)
        The id of the map point each keypoint matched in the last candidate tried, by word or
        by its searches by projection; -1 where none (all -1 if no candidate was tried).
    """

    tracked: Tracked | None
    summary: RelocalizationSummary
    matched: IntArray


@dataclass(frozen=True)
class _Attempt:
    """One candidate tried: the frame if it placed it, how far it got, and its matches."""

    tracked: Tracked | None
    summary: CandidateSummary
    matched: IntArray


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.relocalization`` submodule provides.

    The map, its database (:meth:`slam_it.edge.database.Database.as_dict`), the vocabulary
    (:func:`slam_it.foundations.vocabulary.fields`), the camera, the frame and the settings
    cross as dicts; :class:`Relocalized` as a dict of ``tracked`` (or None), ``matched`` and
    ``summary``, its candidates a list of dicts, RANSAC's termination as its code in
    ``codes()`` (or None) beside ``degenerate``.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def relocalize(  # noqa: PLR0913, PLR0917 -- the map, its database and vocabulary, the frame
        self,
        m: dict[str, Any],
        database: dict[str, Any],
        vocabulary: dict[str, Any],
        camera: dict[str, Any],
        frame: dict[str, Any],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Place a frame in the map through the keyframes that look like it."""
        ...


def ransac_budget(matches: int, options: Options) -> tuple[int, int]:
    """Return the inliers a candidate's pose needs, and the most samples its RANSAC draws.

    MLPnP's ``SetRansacParameters`` (module docstring, step 3).

    Parameters
    ----------
    matches : int
        The candidate's matches by word.
    options : slam_it.edge.options.Options
        ``reloc_min_inliers``, ``reloc_inlier_ratio``, ``reloc_confidence`` and
        ``reloc_ransac_iterations``.

    Returns
    -------
    needed : int
        ``max(reloc_min_inliers, floor(reloc_inlier_ratio * matches))``, and at least P3P's
        sample.
    samples : int
        The samples, from 1 to ``reloc_ransac_iterations``: 1 if every match must be an
        inlier.
    """
    needed = max(
        options.reloc_min_inliers,
        math.floor(options.reloc_inlier_ratio * matches),
        p3p.SAMPLE_SIZE,
    )
    if needed >= matches:
        return needed, 1
    ratio = max(options.reloc_inlier_ratio, needed / matches)
    bound = ransac.required_iterations(ratio, p3p.SAMPLE_SIZE, options.reloc_confidence)
    return needed, max(1, min(math.ceil(bound), options.reloc_ransac_iterations))


def search_keyframe(  # noqa: PLR0913 -- the map, its camera, the frame, the keyframe, the search
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    keyframe: int,
    *,
    skip: IntArray,
    window: float,
    distance: int,
    options: Options,
) -> IntArray:
    """Search a keyframe's points in a frame by projection (ORB-SLAM's keyframe-to-frame search).

    Each point the keyframe's keypoints observe, but not one of `skip`, is projected with the
    frame's pose; in front of the camera, in the image, and within its distances
    (:func:`slam_it.edge.map.in_distance_range`), it is matched among the free keypoints
    within ``window`` times the predicted level's scale, on that level and the two beside it,
    within `distance` (no ratio test), the rotation check between its observation in the
    keyframe and the keypoint.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The frame.
    tracked : slam_it.edge.frames.Tracked
        Its pose, and the keypoints already matched (``points >= 0``), never candidates.
    keyframe : int
        The keyframe's id.
    skip : ndarray of int64
        The ids of points not searched (those already found).
    window : float
        The window's half-width, in pixels at level 0.
    distance : int
        The largest Hamming distance a match may have.
    options : slam_it.edge.options.Options
        The distance margins.

    Returns
    -------
    ndarray of int64, shape (N,)
        The id of the point each keypoint matched by this search, -1 where none.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the keyframe is not in the map.
    """
    k = _keyframe_row(m, keyframe)
    rows = np.arange(m.kf_start[k], m.kf_start[k + 1], dtype=np.int64)
    rows = rows[(m.kp_point[rows] >= 0) & ~np.isin(m.kp_point[rows], skip)]
    points = m.point_rows(m.kp_point[rows])
    pixels, keep = camera.project_in_view(maps.to_camera(tracked.pose, m.pt_xyz[points]))
    gap = np.linalg.norm(m.pt_xyz[points] - maps.camera_centre(tracked.pose), axis=1)
    keep &= maps.in_distance_range(m, points, gap, options)
    rows, points, pixels, gap = rows[keep], points[keep], pixels[keep], gap[keep]
    level = maps.predict_level(m, points, gap)
    projections = matching.Projections(
        xy=pixels,
        radius=np.asarray(window * m.scale_factor**level, dtype=np.float64),
        min_level=level - 1,
        max_level=level + 1,
        descriptors=m.pt_descriptor[points],
        angle=m.kp_angle[rows],
        right=np.full(len(rows), np.nan),
    )
    searched = matching.SearchOptions(max_distance=distance, ratio=1.0, check_orientation=True)
    found = frames.search(frame, projections, searched, taken=tracked.points >= 0)
    out = np.full(frame.size, -1, dtype=np.int64)
    out[found.pairs[:, 1]] = m.pt_id[points[found.pairs[:, 0]]]
    return out


def _keyframe_row(m: Map, keyframe: int) -> int:
    """Return a keyframe's row; refuse one the map does not hold."""
    k = int(m.keyframe_rows(np.array([keyframe], dtype=np.int64))[0])
    if k < 0:
        raise InvalidInputError(f"keyframe {keyframe} is not in the map")
    return k


def _merged(tracked: Tracked, added: IntArray) -> Tracked:
    """Add a search's matches to a frame's."""
    return Tracked(pose=tracked.pose, points=np.where(added >= 0, added, tracked.points))


def _project_more(  # noqa: PLR0913 -- the candidate's state
    m: Map,
    camera: Camera,
    frame: Frame,
    keyframe: int,
    *,
    refined: Tracked,
    found: IntArray,
    options: Options,
) -> tuple[Tracked, IntArray]:
    """Search the candidate's other points twice, optimising when they bring enough inliers.

    Return the frame as last optimised, and the matches the last optimisation started from, or
    would have: those it was given and the searches' additions. `found` is the points RANSAC's
    inliers matched, which the first search skips.
    """
    needed = options.recovery_tracked
    added = search_keyframe(
        m, camera, frame, refined, keyframe,
        skip=found, window=options.reloc_window, distance=options.th_high, options=options,
    )  # fmt: skip
    merged = _merged(refined, added)
    if refined.inliers + int(np.count_nonzero(added >= 0)) < needed:
        return refined, merged.points
    again = tracking.optimize_pose(m, camera, frame, merged, options)
    if not options.reloc_narrow_inliers < again.inliers < needed:
        return again, merged.points
    # The second optimisation's outliers keep their matches until the last (ORB-SLAM's).
    held = Tracked(pose=again.pose, points=merged.points)
    narrow = search_keyframe(
        m, camera, frame, held, keyframe,
        skip=merged.points[merged.points >= 0], window=options.reloc_narrow_window,
        distance=options.reloc_narrow_distance, options=options,
    )  # fmt: skip
    if again.inliers + int(np.count_nonzero(narrow >= 0)) < needed:
        return again, merged.points
    last = _merged(held, narrow)
    return tracking.optimize_pose(m, camera, frame, last, options), last.points


def _match_by_word(  # noqa: PLR0913, PLR0917 -- the map, its database, the frame, the candidate
    m: Map, db: Database, frame: Frame, nodes: IntArray, keyframe: int, options: Options
) -> tuple[IntArray, IntArray]:
    """Match the frame's keypoints to a candidate's points by word (module docstring, step 2).

    Returns
    -------
    pairs : ndarray of int64, shape (n, 2)
        Each match's keyframe keypoint row and frame keypoint, the frame's bearing in front.
    matched : ndarray of int64, shape (F,)
        The point each of the frame's keypoints matched, -1 for none.
    """
    k = _keyframe_row(m, keyframe)
    start = int(m.kf_start[k])
    rows = np.arange(start, m.kf_start[k + 1], dtype=np.int64)
    if len(db.nodes(keyframe)) != len(rows):
        raise InvalidInputError(f"database: keyframe {keyframe}'s features are not the map's")
    rows = rows[m.kp_point[rows] >= 0]
    found = matching.search_by_bow(
        m.kp_descriptor[rows],
        db.nodes(keyframe)[rows - start],
        frame.features.descriptors,
        nodes,
        m.kp_angle[rows],
        frame.features.keypoints.angle,
        matching.Options(
            max_distance=options.th_low, ratio=options.relocalize_ratio, check_orientation=True
        ),
    )
    pairs = found.pairs[frame.bearings[found.pairs[:, 1], 2] > 0.0]  # NaN bearings fail too
    matched = np.full(frame.size, -1, dtype=np.int64)
    matched[pairs[:, 1]] = m.kp_point[rows[pairs[:, 0]]]
    return pairs, matched


def _try(  # noqa: PLR0913 -- the map, the database, the frame, the candidate, the settings
    m: Map,
    db: Database,
    camera: Camera,
    frame: Frame,
    *,
    nodes: IntArray,
    keyframe: int,
    score: float,
    options: Options,
) -> _Attempt:
    """Try to place the frame through one candidate (module docstring, steps 2 to 4).

    Matched by word (:func:`_match_by_word`), placed by PnP RANSAC within MLPnP's budget
    (:func:`ransac_budget`), the pose refined and, short of ``recovery_tracked`` inliers, the
    keyframe's other points searched for (:func:`_project_more`).
    """
    pairs, matched = _match_by_word(m, db, frame, nodes, keyframe, options)
    summary = CandidateSummary(keyframe, score, matches=len(pairs))
    needed, samples = ransac_budget(len(pairs), options)
    if len(pairs) < max(options.reloc_min_matches, needed):
        return _Attempt(None, summary, matched)
    settings = ransac.Options(
        threshold=options.pnp_threshold / camera.focal,
        confidence=options.reloc_confidence,
        max_iterations=samples,
        seed=options.seed,
    )
    keypoints = pairs[:, 1]
    run = tracking.run_pnp(
        m.pt_xyz[m.point_rows(matched[keypoints])], frame.bearings[keypoints], settings
    )
    summary = replace(summary, **run.recorded())
    consensus = run.kept(needed)
    if consensus is None:
        return _Attempt(None, summary, matched)
    pose, inliers = consensus
    sample = np.full(frame.size, -1, dtype=np.int64)
    sample[keypoints[inliers]] = matched[keypoints[inliers]]
    refined = tracking.optimize_pose(m, camera, frame, Tracked(pose=pose, points=sample), options)
    summary = replace(summary, refined=refined.inliers)
    if refined.inliers < options.reloc_min_inliers:
        return _Attempt(None, summary, matched)
    last = refined
    if refined.inliers < options.recovery_tracked:
        last, searched = _project_more(
            m, camera, frame, keyframe, refined=refined, found=sample[sample >= 0], options=options
        )
        matched = np.where(searched >= 0, searched, matched)
    summary = replace(summary, found=last.inliers)
    placed = last if last.inliers >= options.recovery_tracked else None
    return _Attempt(placed, summary, matched)


def _relocalize(
    m: Map, db: Database, camera: Camera, frame: Frame, options: Options
) -> Relocalized:
    """Relocalise a frame (reference)."""
    words, nodes = vocabulary.quantize(db.vocabulary, frame.features.descriptors)
    vector = vocabulary.bow(db.vocabulary, words)
    found = database.candidates(db, m, vector, options)
    summary = RelocalizationSummary(len(vector[0]), sharing=found.sharing, scored=found.scored)
    matched = np.full(frame.size, -1, dtype=np.int64)
    tried: list[CandidateSummary] = []
    for keyframe, score in zip(found.keyframes.tolist(), found.scores.tolist(), strict=True):
        attempt = _try(
            m, db, camera, frame, nodes=nodes, keyframe=keyframe, score=score, options=options
        )
        tried.append(attempt.summary)
        matched = attempt.matched
        if attempt.tracked is not None:
            placed = replace(summary, candidates=tuple(tried), placed=True)
            return Relocalized(attempt.tracked, placed, matched)
    return Relocalized(None, replace(summary, candidates=tuple(tried)), matched)


def relocalize(  # noqa: PLR0913 -- the map, its database, the camera, the frame, the settings
    m: Map,
    db: Database,
    camera: Camera,
    frame: Frame,
    options: Options,
    *,
    backend: str = "python",
) -> Relocalized:
    """Place a frame in the map through the keyframes that look like it (the module docstring).

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    db : slam_it.edge.database.Database
        Its keyframe database, holding every keyframe of the map.
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The frame.
    options : slam_it.edge.options.Options
        The ``reloc_*`` settings, ``th_low``, ``th_high``, ``relocalize_ratio``,
        ``pnp_threshold``, ``recovery_tracked``, the pose optimisation's and ``seed``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Relocalized
        The frame placed with ``recovery_tracked`` inliers or more, or none; how the search
        went; and each keypoint's match in the last candidate tried.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a candidate is not in the map, or its features are not the database's.
    """
    return backends.call(
        backend,
        __name__,
        "relocalize",
        reference=lambda: _relocalize(m, db, camera, frame, options),
        arguments={
            "m": m.as_dict(),
            "database": db.as_dict(),
            "vocabulary": vocabulary.fields(db.vocabulary),
            "camera": camera.as_dict(),
            "frame": frame.as_dict(),
            "options": options.as_dict(),
        },
        decode=_decode_relocalized,
    )


def _decode_candidate(raw: dict[str, Any], codes: backends.Codes) -> CandidateSummary:
    """Rebuild a candidate's summary from a compiled backend's dict."""
    code = raw["termination"]
    termination = (
        None
        if code is None
        else ransac.Termination(backends.decode_name(codes, "ransac_termination", code))
    )
    return CandidateSummary(
        keyframe=int(raw["keyframe"]),
        score=float(raw["score"]),
        matches=int(raw["matches"]),
        iterations=_optional(raw["iterations"]),
        termination=termination,
        degenerate=bool(raw["degenerate"]),
        inliers=_optional(raw["inliers"]),
        refined=_optional(raw["refined"]),
        found=_optional(raw["found"]),
    )


def _optional(raw: Any) -> int | None:  # noqa: ANN401 -- a compiled backend's value
    """Return an optional count as an int or None."""
    return None if raw is None else int(raw)


def _decode_relocalized(raw: dict[str, Any], codes: backends.Codes) -> Relocalized:
    """Rebuild :class:`Relocalized` from a compiled backend's dict."""
    summary = raw["summary"]
    return Relocalized(
        tracked=frames.decode_tracked(raw["tracked"], codes),
        summary=RelocalizationSummary(
            words=int(summary["words"]),
            sharing=int(summary["sharing"]),
            scored=int(summary["scored"]),
            candidates=tuple(_decode_candidate(c, codes) for c in summary["candidates"]),
            placed=bool(summary["placed"]),
        ),
        matched=backends.checked_array(raw["matched"], np.int64, 1, "relocalize matched"),
    )
