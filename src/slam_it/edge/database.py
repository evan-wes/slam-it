"""The keyframe database: each keyframe's words, and which keyframes hold each word.

ORB-SLAM's ``KeyFrameDatabase`` (Mur-Artal and Tardós, T-RO 2017; Gálvez-López and Tardós,
T-RO 2012), for the map being tracked:

- **Each keyframe's words,** made when it enters the map (ORB-SLAM's ``KeyFrame::ComputeBoW``):
  its features quantised (:func:`slam_it.foundations.vocabulary.quantize`), its word vector
  (:func:`slam_it.foundations.vocabulary.bow`), and each feature's node at the direct index's
  level, which matching by word reads (:func:`slam_it.foundations.matching.search_by_bow`).
- **The inverted index:** each word to the keyframes whose vector holds it, by id ascending
  (``mvInvertedFile``). A keyframe leaves it when culled; a map's database goes with the map.
- **Relocalisation's candidates** (:func:`candidates`, ``DetectRelocalizationCandidates``): the
  keyframes that share a word with a frame, found through the index; those sharing more than
  ``relocalization.common_words`` of the most words any shares are scored (the L1 score); each score
  is accumulated over the keyframe's best ``relocalization.neighbours`` covisible keyframes that
  were scored too, the best-scored of the group standing for it; and the groups above
  ``relocalization.group_score`` of the best accumulated score give the candidates, best first.
- **Loop closing's candidates** (:func:`loop_candidates`, ORB-SLAM3's ``DetectNBestCandidates``):
  the same scoring of a keyframe of the map, with ``loops.common_words`` and ``loops.neighbours``,
  leaving out the keyframe and every keyframe linked to it; the best ``loops.candidates`` groups
  give the candidates, with no cut on their scores.
- **A merge's candidates** (:func:`merge_candidates`, ``DetectNBestCandidates``' merge list):
  the same scoring of a keyframe's words in each of the other maps' databases
  (:class:`Indexed`), each map's words counted within it (the online-SLAM design, section 14,
  item 2: ORB-SLAM3 counts them across its atlas, so that a strong match in one map can raise
  the cut past every candidate of another); every map's best ``loops.candidates`` groups,
  then the best ``loops.candidates`` of them all.

**Covisibility** is the graph's (:func:`slam_it.edge.covisibility.links`): a keyframe's best
are its links, those sharing at least ``covisibility.min_points`` points, the most shared
first (ties: the lower id), or its single best when none shares that many, as ORB-SLAM's
``GetBestCovisibilityKeyFrames`` gives them.

**Unlike ORB-SLAM,** a neighbour that shares a word with the frame but was not scored adds
nothing to its group (ORB-SLAM adds the score it kept from an earlier frame's query, which the
three languages could not reproduce alike), and the candidates come best first, ties by id,
where ORB-SLAM keeps the order the index found them in. Scores are float64 sums in a fixed
order: the keyframe's own, then its neighbours' in order.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.edge import covisibility
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations import vocabulary
from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.vocabulary import Vocabulary

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass
class Database:
    """The words of a map's keyframes, and the keyframes that hold each word.

    Attributes
    ----------
    vocabulary : slam_it.foundations.vocabulary.Vocabulary
        The vocabulary the words are of.
    """

    vocabulary: Vocabulary
    _vectors: dict[int, tuple[IntArray, Array]] = field(default_factory=dict, init=False)
    _nodes: dict[int, IntArray] = field(default_factory=dict, init=False)
    _inverted: dict[int, list[int]] = field(default_factory=dict, init=False, repr=False)

    @property
    def keyframes(self) -> IntArray:
        """The keyframes' ids, ascending."""
        return np.array(sorted(self._vectors), dtype=np.int64)

    def add(self, keyframe: int, descriptors: ArrayLike) -> None:
        """Add a keyframe: quantise its features, and index its words.

        Parameters
        ----------
        keyframe : int
            Its id in the map.
        descriptors : array_like of uint8, shape (n, DESCRIPTOR_BYTES)
            Its features' descriptors, in the map's keypoint order.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If the keyframe is already in the database, or the descriptors are malformed.
        """
        if keyframe in self._vectors:
            raise InvalidInputError(f"database: keyframe {keyframe} is already in it")
        words, nodes = vocabulary.quantize(self.vocabulary, descriptors)
        vector = vocabulary.bow(self.vocabulary, words)
        self._vectors[keyframe] = vector
        self._nodes[keyframe] = nodes
        for word in vector[0].tolist():
            bisect.insort(self._inverted.setdefault(word, []), keyframe)

    def erase(self, keyframe: int) -> None:
        """Remove a keyframe (culled): it leaves the index.

        Parameters
        ----------
        keyframe : int
            Its id.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If it is not in the database.
        """
        if keyframe not in self._vectors:
            raise InvalidInputError(f"database: keyframe {keyframe} is not in it")
        ids, _ = self._vectors.pop(keyframe)
        del self._nodes[keyframe]
        for word in ids.tolist():
            holders = self._inverted[word]
            holders.remove(keyframe)
            if not holders:
                del self._inverted[word]

    def vector(self, keyframe: int) -> tuple[IntArray, Array]:
        """Return a keyframe's word vector: its words, ascending, and their values."""
        return self._vectors[keyframe]

    def nodes(self, keyframe: int) -> IntArray:
        """Return each of a keyframe's features' node at the direct index's level."""
        return self._nodes[keyframe]

    def holders(self, word: int) -> list[int]:
        """Return the keyframes whose vector holds a word, ascending."""
        return self._inverted.get(word, [])

    def as_dict(self) -> dict[str, Any]:
        """Return the database as it crosses the bindings, keyframe by keyframe.

        A compiled backend builds its own index from the vectors.

        Returns
        -------
        dict
            ``kf_id`` (K,) ascending; ``bow_start`` (K + 1,) offsets into ``bow_word`` and
            ``bow_value``, each keyframe's vector; ``node_start`` (K + 1,) offsets into
            ``node``, each keyframe's features' nodes. Integers int64, values float64.
        """
        keyframes = sorted(self._vectors)
        vectors = [self._vectors[k] for k in keyframes]
        nodes = [self._nodes[k] for k in keyframes]
        return {
            "kf_id": np.array(keyframes, dtype=np.int64),
            "bow_start": _offsets([len(ids) for ids, _ in vectors]),
            "bow_word": _joined([ids for ids, _ in vectors], np.int64),
            "bow_value": _joined([values for _, values in vectors], np.float64),
            "node_start": _offsets([len(n) for n in nodes]),
            "node": _joined(nodes, np.int64),
        }


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.database`` submodule provides.

    A database crosses as :meth:`Database.as_dict`'s dict, from which the backend builds its
    own index; a frame's word vector as ``ids`` and ``values``; the other maps as a list of
    :meth:`Indexed.as_dict`; :class:`Candidates` and :class:`MergeCandidates` as dicts of their
    fields.
    """

    def candidates(
        self,
        database: dict[str, Any],
        m: dict[str, Any],
        ids: NDArray[np.int64],
        values: NDArray[np.float64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Find the keyframes a frame may be relocalised against."""
        ...

    def loop_candidates(
        self, database: dict[str, Any], m: dict[str, Any], keyframe: int, options: dict[str, Any]
    ) -> dict[str, Any]:
        """Find the keyframes a keyframe may close a loop with."""
        ...

    def merge_candidates(
        self,
        ids: NDArray[np.int64],
        values: NDArray[np.float64],
        others: list[dict[str, Any]],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Find the keyframes of other maps a keyframe may merge its map with."""
        ...


def _offsets(lengths: list[int]) -> IntArray:
    """Return the offsets of consecutive runs of these lengths, from 0."""
    return np.concatenate([[0], np.cumsum(lengths, dtype=np.int64)]).astype(np.int64)


def _joined(parts: list[NDArray[Any]], dtype: type) -> NDArray[Any]:
    """Return the parts end to end, as one array of `dtype`."""
    return np.concatenate(parts).astype(dtype) if parts else np.zeros(0, dtype=dtype)


@dataclass(frozen=True)
class Candidates:
    """Relocalisation's candidates for a frame, best first, and how many keyframes were weighed.

    Attributes
    ----------
    keyframes : ndarray of int64, shape (C,)
        The candidates' ids, by their group's accumulated score, descending (ties: the lower
        id).
    scores : ndarray of float64, shape (C,)
        Each one's group's accumulated score.
    sharing : int
        The keyframes that share a word with the frame.
    scored : int
        Those that shared enough to be scored.
    """

    keyframes: IntArray
    scores: Array
    sharing: int
    scored: int


def _best(m: Map, keyframe: int, count: int, options: Options) -> list[int]:
    """Return a keyframe's best `count` links' ids (by its id), or none.

    A keyframe the map does not hold has none: relocalisation then refuses it by name.
    """
    row = int(m.keyframe_rows(np.array([keyframe]))[0])
    if row < 0:
        return []
    rows = covisibility.links(m, row, options)[0][:count]
    return [int(k) for k in m.kf_id[rows]]


@dataclass(frozen=True)
class _Scoring:
    """A query's scoring: its groups, best first, and how many keyframes were weighed."""

    groups: list[tuple[float, int]]
    sharing: int
    scored: int


def _scoring(  # noqa: PLR0913 -- the database, its map, the query, its exclusions and rules
    db: Database,
    m: Map,
    frame: tuple[IntArray, Array],
    *,
    excluded: set[int],
    rules: tuple[float, int],
    options: Options,
) -> _Scoring:
    """Score the keyframes sharing a frame's words, and accumulate them by group.

    The keyframes sharing a word, but the `excluded`; those sharing more than `common_words` of
    the most words any of them shares, scored; each score accumulated over the keyframe's best
    `neighbours` links that were scored too, the best-scored standing for the group, with
    `rules` the ``(common_words, neighbours)``. The groups come by accumulated score,
    descending, ties by the lower id.
    """
    common_words, neighbours = rules
    shared: dict[int, int] = {}
    for word in frame[0].tolist():
        for keyframe in db.holders(word):
            if keyframe not in excluded:
                shared[keyframe] = shared.get(keyframe, 0) + 1
    if not shared:
        return _Scoring(groups=[], sharing=0, scored=0)
    least = math.floor(common_words * max(shared.values()))
    scores = {
        keyframe: vocabulary.score(frame, db.vector(keyframe))
        for keyframe in sorted(shared)
        if shared[keyframe] > least
    }
    groups: list[tuple[float, int]] = []
    for keyframe, own in scores.items():
        total, best, best_score = own, keyframe, own
        for other in _best(m, keyframe, neighbours, options):
            if other in scores:
                total += scores[other]
                if scores[other] > best_score:
                    best, best_score = other, scores[other]
        groups.append((total, best))
    groups.sort(key=lambda group: (-group[0], group[1]))
    return _Scoring(groups=groups, sharing=len(shared), scored=len(scores))


def _chosen(groups: list[tuple[float, int]], *, above: float, most: int) -> Candidates:
    """Return the groups' best keyframes, each once, scoring above `above`, at most `most`."""
    chosen: dict[int, float] = {}
    for total, best in groups:
        if len(chosen) == most:
            break
        if total > above and best not in chosen:
            chosen[best] = total
    return Candidates(
        keyframes=np.array(list(chosen), dtype=np.int64),
        scores=np.array(list(chosen.values()), dtype=np.float64),
        sharing=0,
        scored=0,
    )


def _candidates(
    db: Database, m: Map, frame: tuple[IntArray, Array], options: Options
) -> Candidates:
    """Find relocalisation's candidates (reference)."""
    settings = options.relocalization
    found = _scoring(
        db, m, frame, excluded=set(), rules=(settings.common_words, settings.neighbours),
        options=options,
    )  # fmt: skip
    if not found.groups:
        return Candidates(np.zeros(0, dtype=np.int64), np.zeros(0), sharing=0, scored=0)
    retained = settings.group_score * found.groups[0][0]
    chosen = _chosen(found.groups, above=retained, most=len(found.groups))
    return replace(chosen, sharing=found.sharing, scored=found.scored)


def _loop_candidates(db: Database, m: Map, keyframe: int, options: Options) -> Candidates:
    """Find loop closing's candidates (reference)."""
    settings = options.loops
    row = int(m.keyframe_rows(np.array([keyframe]))[0])
    linked = {int(k) for k in m.kf_id[covisibility.links(m, row, options)[0]]}
    rules = (settings.common_words, settings.neighbours)
    found = _scoring(
        db, m, db.vector(keyframe), excluded={keyframe} | linked, rules=rules, options=options
    )
    chosen = _chosen(found.groups, above=-math.inf, most=settings.candidates)
    return replace(chosen, sharing=found.sharing, scored=found.scored)


def _decode_candidates(raw: dict[str, Any], codes: backends.Codes) -> Candidates:
    """Rebuild :class:`Candidates` from a compiled backend's dict, its arrays checked."""
    del codes  # candidates cross no codes
    return Candidates(
        keyframes=backends.checked_array(raw["keyframes"], np.int64, 1, "candidates keyframes"),
        scores=backends.checked_array(raw["scores"], np.float64, 1, "candidates scores"),
        sharing=int(raw["sharing"]),
        scored=int(raw["scored"]),
    )


def candidates(
    db: Database,
    m: Map,
    frame: tuple[ArrayLike, ArrayLike],
    options: Options,
    *,
    backend: str = "python",
) -> Candidates:
    """Find the keyframes a frame may be relocalised against (``DetectRelocalizationCandidates``).

    Parameters
    ----------
    db : Database
        The map's keyframe database.
    m : slam_it.edge.map.Map
        The map, whose observations give covisibility (:mod:`slam_it.edge.covisibility`).
    frame : tuple of array_like
        The frame's word vector (:func:`slam_it.foundations.vocabulary.bow`): its words,
        ascending, and their values.
    options : slam_it.edge.options.Options
        ``relocalization.common_words``, ``relocalization.neighbours``,
        ``relocalization.group_score`` and ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Candidates
        The candidates, best first, with their scores; none if no keyframe shares a word.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the vector is malformed (:func:`slam_it.foundations.vocabulary.as_vector`).
    """
    vector = vocabulary.as_vector(*frame)
    return backends.call(
        backend,
        __name__,
        "candidates",
        reference=lambda: _candidates(db, m, vector, options),
        arguments={
            "database": db.as_dict(),
            "m": m.as_dict(),
            "ids": vector[0],
            "values": vector[1],
            "options": options.as_dict(),
        },
        decode=_decode_candidates,
    )


def loop_candidates(
    db: Database, m: Map, keyframe: int, options: Options, *, backend: str = "python"
) -> Candidates:
    """Find the keyframes a keyframe may close a loop with (ORB-SLAM3's ``DetectNBestCandidates``).

    Relocalisation's scoring (:func:`candidates`) of the keyframe's own words, with
    ``loops.common_words`` and ``loops.neighbours``, leaving out the keyframe and every keyframe
    linked to it (:func:`slam_it.edge.covisibility.links`); the best ``loops.candidates`` groups,
    with no cut on their scores.

    Parameters
    ----------
    db : Database
        The map's keyframe database, holding the keyframe.
    m : slam_it.edge.map.Map
        The map, holding the keyframe, whose observations give covisibility.
    keyframe : int
        The keyframe's id.
    options : slam_it.edge.options.Options
        ``loops.candidates``, ``loops.common_words``, ``loops.neighbours`` and
        ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Candidates
        The candidates, best first, with their scores; none if no keyframe outside its links
        shares a word. ``sharing`` and ``scored`` count those keyframes only.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the keyframe is not in the map, or not in the database.
    """
    if m.keyframe_rows(np.array([keyframe], dtype=np.int64))[0] < 0:
        raise InvalidInputError(f"keyframe {keyframe} is not in the map")
    if keyframe not in db.keyframes:
        raise InvalidInputError(f"database: keyframe {keyframe} is not in it")
    return backends.call(
        backend,
        __name__,
        "loop_candidates",
        reference=lambda: _loop_candidates(db, m, keyframe, options),
        arguments={
            "database": db.as_dict(),
            "m": m.as_dict(),
            "keyframe": keyframe,
            "options": options.as_dict(),
        },
        decode=_decode_candidates,
    )


@dataclass(frozen=True)
class Indexed:
    """A map with its keyframe database, and its number among a run's maps.

    Attributes
    ----------
    number : int
        The map's number: maps are numbered in the order they started.
    map : slam_it.edge.map.Map
        The map.
    database : Database
        Its keyframe database, holding every keyframe of the map.
    """

    number: int
    map: Map
    database: Database

    def as_dict(self) -> dict[str, Any]:
        """Return the map, its database and number as they cross the bindings.

        Returns
        -------
        dict
            ``number``, ``m`` (:meth:`slam_it.edge.map.Map.as_dict`) and ``database``
            (:meth:`Database.as_dict`).
        """
        return {"number": self.number, "m": self.map.as_dict(), "database": self.database.as_dict()}


@dataclass(frozen=True)
class MergeCandidates:
    """A merge's candidates, from every other map, best first, and how many were weighed.

    Attributes
    ----------
    maps : ndarray of int64, shape (C,)
        Each candidate's map number.
    keyframes : ndarray of int64, shape (C,)
        Its id in that map.
    scores : ndarray of float64, shape (C,)
        Its group's accumulated score, descending (ties: the lower map number, then the lower
        id).
    sharing : int
        The keyframes, over every map, that share a word with the query.
    scored : int
        Those scored.
    """

    maps: IntArray
    keyframes: IntArray
    scores: Array
    sharing: int
    scored: int


def merge_candidates(
    vector: tuple[ArrayLike, ArrayLike],
    others: Sequence[Indexed],
    options: Options,
    *,
    backend: str = "python",
) -> MergeCandidates:
    """Find the keyframes of other maps a keyframe may merge its map with.

    ORB-SLAM3's ``DetectNBestCandidates``' merge list, with each map's words counted within it
    (the module docstring): in each map, :func:`loop_candidates`' scoring of the query's words
    (``loops.common_words``, ``loops.neighbours``), nothing left out, its best
    ``loops.candidates`` groups; then the best ``loops.candidates`` of every map's, by
    accumulated score (comparable across maps, all scored under one vocabulary).

    Parameters
    ----------
    vector : tuple of array_like
        The query keyframe's word vector: its words, ascending, and their values.
    others : sequence of Indexed
        The maps it may merge with, each with its database (not the query's own map).
    options : slam_it.edge.options.Options
        ``loops.candidates``, ``loops.common_words``, ``loops.neighbours`` and
        ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    MergeCandidates
        The candidates, best first; none if no keyframe of another map shares a word.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the vector is malformed, or two maps share a number.
    """
    query = vocabulary.as_vector(*vector)
    numbers = [other.number for other in others]
    if len(set(numbers)) != len(numbers):
        raise InvalidInputError("others: expected each map once, by its number")
    return backends.call(
        backend,
        __name__,
        "merge_candidates",
        reference=lambda: _merge_candidates(query, others, options),
        arguments={
            "ids": query[0],
            "values": query[1],
            "others": [other.as_dict() for other in others],
            "options": options.as_dict(),
        },
        decode=_decode_merge_candidates,
    )


def _merge_candidates(
    query: tuple[IntArray, Array], others: Sequence[Indexed], options: Options
) -> MergeCandidates:
    """Find a merge's candidates (reference)."""
    settings = options.loops
    found: list[tuple[float, int, int]] = []
    sharing = scored = 0
    for other in others:
        scoring = _scoring(
            other.database, other.map, query, excluded=set(),
            rules=(settings.common_words, settings.neighbours), options=options,
        )  # fmt: skip
        sharing += scoring.sharing
        scored += scoring.scored
        chosen = _chosen(scoring.groups, above=-math.inf, most=settings.candidates)
        pairs = zip(chosen.keyframes.tolist(), chosen.scores.tolist(), strict=True)
        found.extend((score, other.number, keyframe) for keyframe, score in pairs)
    found.sort(key=lambda c: (-c[0], c[1], c[2]))
    best = found[: settings.candidates]
    return MergeCandidates(
        maps=np.array([c[1] for c in best], dtype=np.int64),
        keyframes=np.array([c[2] for c in best], dtype=np.int64),
        scores=np.array([c[0] for c in best], dtype=np.float64),
        sharing=sharing,
        scored=scored,
    )


def _decode_merge_candidates(raw: dict[str, Any], codes: backends.Codes) -> MergeCandidates:
    """Rebuild :class:`MergeCandidates` from a compiled backend's dict, its arrays checked."""
    del codes  # the candidates cross no codes
    return MergeCandidates(
        maps=backends.checked_array(raw["maps"], np.int64, 1, "merge candidates maps"),
        keyframes=backends.checked_array(
            raw["keyframes"], np.int64, 1, "merge candidates keyframes"
        ),
        scores=backends.checked_array(raw["scores"], np.float64, 1, "merge candidates scores"),
        sharing=int(raw["sharing"]),
        scored=int(raw["scored"]),
    )
