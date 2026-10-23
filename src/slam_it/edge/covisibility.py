"""The covisibility graph: keyframes linked by the points they share, as ORB-SLAM's are.

ORB-SLAM's covisibility graph and spanning tree (Mur-Artal et al., T-RO 2015, sections III.D and
V.D; ``KeyFrame::UpdateConnections``, ``KeyFrame::SetBadFlag``, ``Tracking::UpdateLocalKeyFrames``):

- **The weights,** counted when asked: two keyframes' weight is the number of points both
  observe, counted from the map's observations each time it is asked for, so it is always
  exact. ORB-SLAM stores the weights and updates a keyframe's at a few points (at
  initialisation, when it is inserted, after fusion and in loop closing), so they are stale in
  between: an observation bundle adjustment drops still counts.
- **The links** (:func:`links`): a keyframe is linked to those it shares at least
  ``covisibility.min_points`` (15) with, or, if it shares that many with none, to the one it
  shares most with; its links run most shared first, ties broken by the lower id. Its best are
  the first of its links.
- **The spanning tree,** the one part stored: each keyframe's parent
  (:attr:`slam_it.edge.map.Map.kf_parent`) is the keyframe it shared the most points with when
  it was inserted (:func:`adopt`, ORB-SLAM's first ``UpdateConnections``); a map's first
  keyframe has none. A keyframe leaves the map only by :func:`cull`, ORB-SLAM's ``SetBadFlag``:
  its children are given new parents (:func:`reparent`), then it is erased, and
  :func:`slam_it.edge.map.erase_keyframe` refuses a keyframe that is still a parent. ``cull``
  spares a keyframe with a loop or merge edge, and one loop detection holds, as ORB-SLAM's
  ``SetNotErase`` does.
- **Tracking's local map** (:func:`local_map`, ``UpdateLocalKeyFrames``): K1, the keyframes
  that observe the points a frame was matched to, the most shared first, the first of them
  the frame's reference keyframe; then, for each of K1 in that order, while the local map holds
  no more than ``covisibility.local_keyframes``: the first of its best
  ``covisibility.local_neighbours`` not yet in it, its first child not yet in it, and its
  parent if not yet in it.
- **The strong links** (:func:`strong_links`): every pair of keyframes sharing at least
  ``covisibility.strong_points`` (100), the essential graph's (``GetCovisiblesByWeight``), drawn
  by the viewer.

**Unlike ORB-SLAM,** the local map expands every K1 keyframe, as the papers describe K2:
ORB-SLAM2's and ORB-SLAM3's loops end the whole expansion once a parent is added (a ``break``
out of the outer loop), and take K1 in the order of its keyframes' addresses in memory; here
K1 runs most shared first, ties by id, in the three languages alike. And ORB-SLAM's source keeps
every keyframe sharing a point in the list it re-sorts (``UpdateBestCovisibles``), with no
threshold of 15: ``SetBadFlag`` can join a child to a candidate it shares fewer with, where here
it takes the culled keyframe's parent, and ORB-SLAM3's culling tests every keyframe sharing a
point with the new one, up to 100. Here the paper's rule holds in every stage. The design is
section 8 of ``docs/design/2026-10-14-online-slam.md``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.edge import map as maps
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations.arrays import holds_integers
from slam_it.foundations.errors import InvalidInputError

IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class LocalMap:
    """Tracking's local map: its keyframes and the frame's reference keyframe.

    Attributes
    ----------
    keyframes : ndarray of int64, shape (L,)
        The local keyframes' rows: K1, the most shared first, then K2 in the order added.
    reference : int
        The reference keyframe's row (K1's first), or -1 when the frame was matched to no point.
    """

    keyframes: IntArray
    reference: int


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.covisibility`` submodule provides."""

    def links(
        self, m: dict[str, Any], k: int, options: dict[str, Any]
    ) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
        """Return a keyframe's links' rows and weights."""
        ...

    def local_map(
        self, m: dict[str, Any], points: NDArray[np.int64], options: dict[str, Any]
    ) -> dict[str, Any]:
        """Return tracking's local map, as a dict of ``keyframes`` and ``reference``."""
        ...

    def adopt(self, m: dict[str, Any], k: int) -> dict[str, Any]:
        """Return the map with a keyframe given its parent."""
        ...

    def reparent(self, m: dict[str, Any], k: int, options: dict[str, Any]) -> dict[str, Any]:
        """Return the map with a keyframe's children given new parents."""
        ...

    def cull(
        self, m: dict[str, Any], k: int, options: dict[str, Any], held: NDArray[np.int64]
    ) -> dict[str, Any]:
        """Return the map without a keyframe, its children given new parents first."""
        ...

    def strong_links(self, m: dict[str, Any], options: dict[str, Any]) -> NDArray[np.int64]:
        """Return the strong links, (L, 3)."""
        ...

    def view_keyframes(
        self,
        m: dict[str, Any],
        last: NDArray[np.int64],
        reference: int,
        options: dict[str, Any],
    ) -> NDArray[np.int64]:
        """Return the keyframes a view holds, by row."""
        ...


# --- reference ----------------------------------------------------------------------------


def _unique_pairs(m: Map, rows: IntArray) -> IntArray:
    """Return the observations at keypoint `rows`, each once, as (point id, keyframe row), (P, 2).

    Sorted by point, so each point's observations are a run of rows.
    """
    seen = np.column_stack([m.kp_point[rows], m.keypoint_keyframe()[rows]])
    return np.unique(seen, axis=0).reshape(-1, 2)


def _pairs(m: Map) -> IntArray:
    """Return every observation of the map once (:func:`_unique_pairs`)."""
    return _unique_pairs(m, np.flatnonzero(m.kp_point >= 0))


def _runs(starts: IntArray, sizes: IntArray) -> IntArray:
    """Return the indices ``starts[i]`` to ``starts[i] + sizes[i] - 1``, run after run."""
    return np.repeat(starts - np.cumsum(sizes) + sizes, sizes) + np.arange(int(sizes.sum()))


def _weights(m: Map) -> IntArray:
    """Every pair of keyframes' shared points, by row, (K, K), 0 on the diagonal.

    For the strong links, which ask for every pair; one keyframe's row is :func:`_shared`.
    """
    pairs = _pairs(m)
    out = np.zeros((m.keyframes, m.keyframes), dtype=np.int64)
    if len(pairs) == 0:
        return out
    _, starts, sizes = np.unique(pairs[:, 0], return_index=True, return_counts=True)
    # Each observation of a point paired with every observation of it: (keyframe, keyframe).
    point = np.repeat(np.arange(len(starts)), sizes)
    left = np.repeat(pairs[:, 1], sizes[point])
    right = pairs[_runs(starts[point], sizes[point]), 1]
    np.add.at(out, (left, right), 1)
    np.fill_diagonal(out, 0)
    return out


def _observing(m: Map, pairs: IntArray, points: IntArray) -> IntArray:
    """Return how many of the points (ids) each keyframe observes, by row, from `pairs`."""
    observed = pairs[np.isin(pairs[:, 0], points), 1]
    return np.bincount(observed, minlength=m.keyframes).astype(np.int64)


def _shared(m: Map, k: int) -> IntArray:
    """Return the points keyframe row `k` shares with each keyframe, by row, 0 for itself.

    From its own points' observations alone, for a caller reading one row.
    """
    seen = m.kp_point[m.keypoint_rows(k)]
    own = np.unique(seen[seen >= 0])
    out = _observing(m, _unique_pairs(m, np.flatnonzero(np.isin(m.kp_point, own))), own)
    out[k] = 0
    return out


def _shared_in(m: Map, pairs: IntArray, k: int) -> IntArray:
    """Return :func:`_shared` read from the map's observations (:func:`_pairs`).

    For a caller reading many rows, which takes the observations once.
    """
    own = pairs[pairs[:, 1] == k, 0]
    start = np.searchsorted(pairs[:, 0], own, side="left")
    sizes = np.searchsorted(pairs[:, 0], own, side="right") - start
    out = np.bincount(pairs[_runs(start, sizes), 1], minlength=m.keyframes).astype(np.int64)
    out[k] = 0
    return out


def _ranked(m: Map, shared: IntArray) -> IntArray:
    """Return the rows sharing a point, most shared first, ties by the lower id."""
    others = np.flatnonzero(shared > 0)
    return others[np.lexsort((m.kf_id[others], -shared[others]))]


def _links_of(m: Map, shared: IntArray, min_points: int) -> IntArray:
    """Return a keyframe's links from its weights: at least `min_points`, or its best one."""
    ranked = _ranked(m, shared)
    kept = ranked[shared[ranked] >= min_points]
    return kept if len(kept) else ranked[:1]


def _links(m: Map, k: int, options: Options) -> tuple[IntArray, IntArray]:
    """Return a keyframe's links and their weights (reference)."""
    shared = _shared(m, k)
    linked = _links_of(m, shared, options.covisibility.min_points)
    return linked, shared[linked]


def _local_map(m: Map, points: IntArray, options: Options) -> LocalMap:
    """Return tracking's local map (reference)."""
    settings = options.covisibility
    pairs = _pairs(m)
    k1 = _ranked(m, _observing(m, pairs, np.unique(points[points >= 0])))
    if len(k1) == 0:
        return LocalMap(keyframes=np.zeros(0, dtype=np.int64), reference=-1)
    local = [int(k) for k in k1]
    inside = set(local)

    def add(row: int) -> None:
        local.append(row)
        inside.add(row)

    for k in k1:
        if len(local) > settings.local_keyframes:
            break
        shared = _shared_in(m, pairs, int(k))
        best = _links_of(m, shared, settings.min_points)[: settings.local_neighbours]
        neighbour = next((int(r) for r in best if int(r) not in inside), None)
        if neighbour is not None:
            add(neighbour)
        children = np.flatnonzero(m.kf_parent == m.kf_id[k])
        child = next((int(r) for r in children if int(r) not in inside), None)
        if child is not None:
            add(child)
        parent = m.keyframe_rows(m.kf_parent[k : k + 1])[0]
        if parent >= 0 and int(parent) not in inside:
            add(int(parent))
    return LocalMap(keyframes=np.array(local, dtype=np.int64), reference=int(k1[0]))


def _view_keyframes(m: Map, last: IntArray, reference: int, options: Options) -> IntArray:
    """Return the keyframes a view holds (reference)."""
    local = _local_map(m, last, options).keyframes
    pairs = _pairs(m)
    first = np.flatnonzero(_observing(m, pairs, m.pt_id[m.observed_by(local)]) > 0)
    inside = np.zeros(m.keyframes, dtype=bool)
    for k in first.tolist():
        inside[k] = True
        inside |= _shared_in(m, pairs, k) > 0
        inside |= m.kf_parent == m.kf_id[k]  # its children
        parent = int(m.keyframe_rows(m.kf_parent[k : k + 1])[0])
        if parent >= 0:
            inside[parent] = True
    if reference >= 0:
        row = int(m.keyframe_rows(np.array([reference], dtype=np.int64))[0])
        if row >= 0:
            inside[row] = True
    return np.flatnonzero(inside).astype(np.int64)


def _adopt(m: Map, k: int) -> Map:
    """Give a keyframe its parent, if it has none yet (reference)."""
    if m.kf_id[k] == 0 or m.kf_parent[k] >= 0:
        return m
    ranked = _ranked(m, _shared(m, k))
    if len(ranked) == 0:
        return m
    parents = m.kf_parent.copy()
    parents[k] = m.kf_id[ranked[0]]
    return replace(m, kf_parent=parents)


def _reparent(m: Map, k: int, options: Options) -> Map:
    """Give a keyframe's children new parents (reference)."""
    children = [int(r) for r in np.flatnonzero(m.kf_parent == m.kf_id[k])]
    if not children:
        return m
    pairs = _pairs(m)
    shared = {child: _shared_in(m, pairs, child) for child in children}
    for counts in shared.values():
        counts[k] = 0  # its links erased first, as SetBadFlag erases them
    parent = int(m.kf_parent[k])
    candidates = {int(r) for r in m.keyframe_rows(np.array([parent])) if r >= 0}
    parents = m.kf_parent.copy()
    while children:
        best: tuple[int, int, int] | None = None  # (weight, child, candidate)
        for child in children:
            for row in _links_of(m, shared[child], options.covisibility.min_points):
                if int(row) in candidates and (best is None or shared[child][row] > best[0]):
                    best = (int(shared[child][row]), child, int(row))
        if best is None:
            break
        _, child, row = best
        parents[child] = m.kf_id[row]
        candidates.add(child)
        children.remove(child)
    parents[children] = parent  # linked to no candidate: its own parent
    return replace(m, kf_parent=parents)


def _cull(m: Map, k: int, options: Options, held: IntArray) -> Map:
    """Give a keyframe's children new parents, then erase it, unless it is spared (reference)."""
    if maps.has_loop(m, k) or maps.has_merge(m, k) or m.kf_id[k] in held:
        return m
    return maps.erase_keyframe(_reparent(m, k, options), k)


def _strong_links(m: Map, options: Options) -> IntArray:
    """Return every pair of keyframes sharing at least the strong points (reference)."""
    weights = _weights(m)
    a, b = np.nonzero(np.triu(weights >= options.covisibility.strong_points, 1))
    return np.column_stack([m.kf_id[a], m.kf_id[b], weights[a, b]]).astype(np.int64)


# --- dispatch -----------------------------------------------------------------------------


def as_points(points: ArrayLike) -> IntArray:
    """Check a frame's points: a vector of point ids, -1 for a keypoint matched to none.

    Parameters
    ----------
    points : array_like of int
        The point each keypoint was matched to.

    Returns
    -------
    ndarray of int64
        The points, checked.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If they are not a vector of ids, -1 or more.
    """
    arr = np.asarray(points)
    if arr.size == 0:
        return np.zeros(0, dtype=np.int64)
    if arr.ndim != 1 or not holds_integers(arr) or arr.min() < -1:
        raise InvalidInputError("points: expected a vector of point ids (-1 or more)")
    return np.ascontiguousarray(arr, dtype=np.int64)


def as_held(held: ArrayLike) -> IntArray:
    """Check the keyframes loop detection holds: a vector of keyframe ids, 0 or more.

    Parameters
    ----------
    held : array_like of int
        The ids.

    Returns
    -------
    ndarray of int64
        The ids, checked.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If they are not a vector of ids.
    """
    arr = np.asarray(held)
    if arr.size == 0:
        return np.zeros(0, dtype=np.int64)
    if arr.ndim != 1 or not holds_integers(arr) or arr.min() < 0:
        raise InvalidInputError("held: expected a vector of keyframe ids")
    return np.ascontiguousarray(arr, dtype=np.int64)


def links(
    m: Map, k: int, options: Options, *, backend: str = "python"
) -> tuple[IntArray, IntArray]:
    """Return a keyframe's links and their weights (``GetVectorCovisibleKeyFrames``).

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map, whose observations give the weights.
    k : int
        The keyframe's row.
    options : slam_it.edge.options.Options
        ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    tuple of ndarray of int64
        The linked keyframes' rows, most shared first (ties: the lower id), and the points each
        shares: those sharing at least ``covisibility.min_points``, or, if none does, the one
        sharing the most; none if no keyframe shares a point.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `k` is not a keyframe row.
    """
    row = maps.as_keyframe_row(m, k)
    return backends.call(
        backend,
        __name__,
        "links",
        reference=lambda: _links(m, row, options),
        arguments={"m": m.as_dict(), "k": row, "options": options.as_dict()},
        decode=lambda raw, codes: (
            backends.checked_array(raw[0], np.int64, 1, "links rows"),
            backends.checked_array(raw[1], np.int64, 1, "links weights"),
        ),
    )


def local_map(m: Map, points: ArrayLike, options: Options, *, backend: str = "python") -> LocalMap:
    """Return tracking's local map, ORB-SLAM's K1 and K2, and the frame's reference keyframe.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    points : array_like of int
        The point each of the frame's keypoints was matched to, -1 for none.
    options : slam_it.edge.options.Options
        ``covisibility.min_points``, ``covisibility.local_keyframes`` and
        ``covisibility.local_neighbours``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    LocalMap
        Its keyframes' rows, K1 then K2, and the reference keyframe's row (K1's first); none,
        and -1, if the frame was matched to no point of the map.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `points` is not a vector of ids.
    """
    checked = as_points(points)
    return backends.call(
        backend,
        __name__,
        "local_map",
        reference=lambda: _local_map(m, checked, options),
        arguments={"m": m.as_dict(), "points": checked, "options": options.as_dict()},
        decode=lambda raw, codes: LocalMap(
            keyframes=backends.checked_array(raw["keyframes"], np.int64, 1, "local keyframes"),
            reference=int(raw["reference"]),
        ),
    )


def view_keyframes(
    m: Map,
    last: ArrayLike,
    reference: int,
    options: Options,
    *,
    backend: str = "python",
) -> IntArray:
    """Return the keyframes tracking's next frame may read, from where the last frame left it.

    What the view a writer publishes holds (the online-SLAM design's section 12, item 3; the
    natives' pipelines in threads of their own), so that tracking on it decides as on the whole
    map: the last frame's local map (:func:`local_map`) gives the points tracking may match, by
    the motion model or the local fallback; then the keyframes observing them (the K1 of any
    frame matching them) and, for each of those, every keyframe sharing a point with it, its
    children and its parent, which K2 and the keyframe decision read; and the last frame's
    reference keyframe. Python's own view is its immutable map, published by reference.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    last : array_like of int
        The point each of the last frame's keypoints tracked, -1 for none.
    reference : int
        Its reference keyframe's id, or -1.
    options : slam_it.edge.options.Options
        The local map's settings (``covisibility``).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (V,)
        The keyframes' rows, ascending.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `last` is not a vector of ids.
    """
    checked = as_points(last)
    return backends.call(
        backend,
        __name__,
        "view_keyframes",
        reference=lambda: _view_keyframes(m, checked, int(reference), options),
        arguments={
            "m": m.as_dict(),
            "last": checked,
            "reference": int(reference),
            "options": options.as_dict(),
        },
        decode=lambda raw, codes: backends.checked_array(raw, np.int64, 1, "view keyframes"),
    )


def adopt(m: Map, k: int, *, backend: str = "python") -> Map:
    """Return the map with a keyframe given its parent, if it has none yet.

    The parent is the keyframe it shares the most points with (ties: the lower id), as
    ORB-SLAM's first ``UpdateConnections`` sets it. A map's first keyframe (id 0) has none, and
    a keyframe that shares no point keeps none until it does. It is for the keyframe being
    inserted, the map's newest, every other already in the tree: given an older keyframe
    without a parent, it may take a newer one whose own chain leads back to it, a cycle.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    k : int
        The keyframe's row.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    slam_it.edge.map.Map
        The map, ``kf_parent`` set for the keyframe if it had none.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `k` is not a keyframe row.
    """
    row = maps.as_keyframe_row(m, k)
    return backends.call(
        backend,
        __name__,
        "adopt",
        reference=lambda: _adopt(m, row),
        arguments={"m": m.as_dict(), "k": row},
        decode=maps.decode,
    )


def reparent(m: Map, k: int, options: Options, *, backend: str = "python") -> Map:
    """Return the map with a keyframe's children given new parents, before it is culled.

    ORB-SLAM's ``SetBadFlag``: its links are dropped, and the candidates start as its parent;
    each round, of the children and the candidates they are linked to, the most strongly linked
    pair is joined (ties: the lower child, then its links' order), and the child becomes a
    candidate; the children linked to no candidate get its parent.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    k : int
        The keyframe's row.
    options : slam_it.edge.options.Options
        ``covisibility.min_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    slam_it.edge.map.Map
        The map, its children's ``kf_parent`` changed; the keyframe itself is not removed.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `k` is not a keyframe row.
    """
    row = maps.as_keyframe_row(m, k)
    return backends.call(
        backend,
        __name__,
        "reparent",
        reference=lambda: _reparent(m, row, options),
        arguments={"m": m.as_dict(), "k": row, "options": options.as_dict()},
        decode=maps.decode,
    )


def cull(m: Map, k: int, options: Options, *, held: ArrayLike = (), backend: str = "python") -> Map:
    """Return the map without a keyframe, its children given new parents first (``SetBadFlag``).

    The one way a keyframe leaves the graph: :func:`reparent`, then
    :func:`slam_it.edge.map.erase_keyframe`, which refuses a keyframe that is still a parent, so
    every parent stays a keyframe of the map. The map's first keyframe (id 0), its tree's root,
    is refused, as ``SetBadFlag`` leaves it. A keyframe with a loop or merge edge, or one of
    `held`, is spared: the map comes back as it was (ORB-SLAM's ``SetNotErase``; ORB-SLAM erases
    a spared keyframe once it is released, where here a later culling weighs it again).

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    k : int
        The keyframe's row.
    options : slam_it.edge.options.Options
        ``covisibility.min_points``.
    held : array_like of int, optional
        The ids of the keyframes loop detection holds (default: none).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    slam_it.edge.map.Map
        The map without the keyframe, its children's ``kf_parent`` changed; or as it was, the
        keyframe spared.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `k` is not a keyframe row, or is the map's first keyframe, or `held` is not ids.
    """
    row = maps.as_keyframe_row(m, k)
    if m.kf_id[row] == 0:
        raise InvalidInputError(
            "k: keyframe 0 is the map's first, its tree's root, and is not culled"
        )
    kept = as_held(held)
    return backends.call(
        backend,
        __name__,
        "cull",
        reference=lambda: _cull(m, row, options, kept),
        arguments={"m": m.as_dict(), "k": row, "options": options.as_dict(), "held": kept},
        decode=maps.decode,
    )


def strong_links(m: Map, options: Options, *, backend: str = "python") -> IntArray:
    """Return every pair of keyframes sharing at least ``covisibility.strong_points``.

    Parameters
    ----------
    m : slam_it.edge.map.Map
        The map.
    options : slam_it.edge.options.Options
        ``covisibility.strong_points``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (L, 3)
        Each pair's ids, the lower first, and the points they share, by the first id then the
        second.
    """
    return backends.call(
        backend,
        __name__,
        "strong_links",
        reference=lambda: _strong_links(m, options),
        arguments={"m": m.as_dict(), "options": options.as_dict()},
        decode=lambda raw, codes: backends.checked_array(raw, np.int64, 2, "strong links"),
    )
