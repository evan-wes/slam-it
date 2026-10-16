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
  ``reloc_common_words`` of the most words any shares are scored (the L1 score); each score is
  accumulated over the keyframe's best ``reloc_neighbours`` covisible keyframes that were scored
  too, the best-scored of the group standing for it; and the groups above
  ``reloc_group_score`` of the best accumulated score give the candidates, best first.

**Covisibility** is counted from the map's observations until milestone 2B's second PR builds
the covisibility graph: two keyframes are covisible when they observe at least
``covisible_points`` of the same points, the best those that share the most (ties: the lower
id). ORB-SLAM keeps a keyframe's single best neighbour when none shares 15; here it has none.

**Unlike ORB-SLAM,** a neighbour that shares a word with the frame but was not scored adds
nothing to its group (ORB-SLAM adds the score it kept from an earlier frame's query, which the
three languages could not reproduce alike), and the candidates come best first, ties by id,
where ORB-SLAM keeps the order the index found them in. Scores are float64 sums in a fixed
order: the keyframe's own, then its neighbours' in order.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
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
    own index; a frame's word vector as ``ids`` and ``values``; :class:`Candidates` as a dict
    of its fields.
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


def best_covisible(m: Map, keyframes: list[int], options: Options) -> dict[int, list[int]]:
    """Return each keyframe's best covisible keyframes, from the map's observations.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    keyframes : list of int
        The keyframes' ids.
    options : slam_it.edge.options.Options
        ``covisible_points`` and ``reloc_neighbours``.

    Returns
    -------
    dict
        Each keyframe's id to the ids of the keyframes that observe at least
        ``covisible_points`` of its points, the most shared first (ties: the lower id), at
        most ``reloc_neighbours``.
    """
    rows = np.flatnonzero(m.kp_point >= 0)
    owner = m.kf_id[m.keypoint_keyframe()[rows]]
    seen = np.unique(np.column_stack([owner, m.kp_point[rows]]), axis=0)  # (keyframe, point)
    out: dict[int, list[int]] = {}
    for keyframe in keyframes:
        mine = seen[seen[:, 0] == keyframe, 1]
        others = seen[np.isin(seen[:, 1], mine) & (seen[:, 0] != keyframe), 0]
        ids, counts = np.unique(others, return_counts=True)
        kept = counts >= options.covisible_points
        ids, counts = ids[kept], counts[kept]
        order = np.lexsort((ids, -counts))[: options.reloc_neighbours]
        out[keyframe] = ids[order].tolist()
    return out


def _candidates(
    db: Database, m: Map, frame: tuple[IntArray, Array], options: Options
) -> Candidates:
    """Find relocalisation's candidates (reference)."""
    shared: dict[int, int] = {}
    for word in frame[0].tolist():
        for keyframe in db.holders(word):
            shared[keyframe] = shared.get(keyframe, 0) + 1
    none = np.zeros(0, dtype=np.int64)
    if not shared:
        return Candidates(none, np.zeros(0), sharing=0, scored=0)
    least = math.floor(options.reloc_common_words * max(shared.values()))
    scores = {
        keyframe: vocabulary.score(frame, db.vector(keyframe))
        for keyframe in sorted(shared)
        if shared[keyframe] > least
    }
    neighbours = best_covisible(m, list(scores), options)
    groups: list[tuple[float, int]] = []
    for keyframe, own in scores.items():
        total, best, best_score = own, keyframe, own
        for other in neighbours[keyframe]:
            if other in scores:
                total += scores[other]
                if scores[other] > best_score:
                    best, best_score = other, scores[other]
        groups.append((total, best))
    retained = options.reloc_group_score * max(total for total, _ in groups)
    chosen: dict[int, float] = {}
    for total, best in sorted(groups, key=lambda group: (-group[0], group[1])):
        if total > retained and best not in chosen:
            chosen[best] = total
    return Candidates(
        keyframes=np.array(list(chosen), dtype=np.int64),
        scores=np.array(list(chosen.values()), dtype=np.float64),
        sharing=len(shared),
        scored=len(scores),
    )


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
        The map, whose observations give covisibility (:func:`best_covisible`).
    frame : tuple of array_like
        The frame's word vector (:func:`slam_it.foundations.vocabulary.bow`): its words,
        ascending, and their values.
    options : slam_it.edge.options.Options
        ``reloc_common_words``, ``reloc_neighbours``, ``reloc_group_score`` and
        ``covisible_points``.
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
