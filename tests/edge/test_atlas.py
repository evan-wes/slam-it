"""The atlas's tests: its records carried through a monocular merge, into the matched map's units.

The two maps (:class:`tests.edge.conftest.TwoMaps`, the monocular variant) are merged as the
pipeline merges them (:func:`slam_it.edge.loops.merge`, then
:meth:`slam_it.edge.atlas.Atlas.merge`), one of the active map's keyframes culled first, its
child given its parent (as :func:`slam_it.edge.covisibility.cull` does), so the atlas holds what
a merge carries: points and a keyframe removed, the culled keyframe's pose relative to its
parent, and frames tracked against the active map's keyframes. The active map's world is the
truth's scaled by :data:`tests.edge.conftest.MERGE_SCALE`: a record left in its units is out of
place by nearly half its offset from its keyframe (``1 / 0.7 - 1``). The merged map, reset, takes
the frames of both maps with it.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
import pytest

from slam_it.edge import loops
from slam_it.edge import map as maps
from slam_it.edge.atlas import Atlas, Trajectory
from slam_it.edge.database import Indexed
from slam_it.edge.map import Map
from slam_it.foundations import sim3
from slam_it.foundations.errors import InvalidInputError
from tests.edge.conftest import SPLIT, TwoMaps
from tests.edge.support import CAMERA, OPTIONS, assert_close

OPTIONS_NO_GLOBAL_BA = OPTIONS.with_settings({"loops.global_ba": False})
"""The merge's settings: global BA off, so the merge's own correction alone places the maps."""

CULLED = 8
"""The active map's keyframe culled before the merge (its row and id)."""

PLACED = ((5, 7), (CULLED, 10))
"""Frames tracked in the active map, each against the first keyframe of a pair (its reference),
where the second is (its pose); the second frame's reference is then culled, so it is placed
through its parent."""

FOUND_AT = -2
"""The active keyframe the merge is found on (``tests.edge.test_merges.FOUND_AT``)."""

PLACE_BOUND = 0.015
"""A frame carried through the merge lies within this of its true place, in metres (measured:
0.8 and 0.1 cm; with its pose relative to its keyframe left in the active map's units, 8.4 and
13.7 cm)."""


@dataclass(frozen=True)
class _Merged:
    """The atlas after the merge, with what it was given."""

    atlas: Atlas
    active: Map
    culled: Map
    merged: loops.Merged
    to_world: np.ndarray
    trajectory: Trajectory


def _merge(two: TwoMaps, *, in_matched: bool = False) -> _Merged:
    """Track frames in the active map, cull a keyframe, and merge it into the matched map.

    With `in_matched`, a frame is tracked in the matched map first, against its first keyframe.
    """
    parents = two.active.kf_parent.copy()
    parents[CULLED + 1] = parents[CULLED]
    active = replace(two.active, kf_parent=parents)
    culled = maps.erase_keyframe(active, CULLED)
    others = [Indexed(0, two.matched, two.matched_database)]
    found = loops.detect_merge(
        culled, two.active_database, CAMERA, int(culled.kf_id[FOUND_AT]), others=others,
        pending=None, mode="monocular", options=OPTIONS_NO_GLOBAL_BA,
    ).merge  # fmt: skip
    assert found is not None
    atlas = Atlas(OPTIONS_NO_GLOBAL_BA)
    atlas.start()
    atlas.date(two.matched)
    if in_matched:
        atlas.place_frame(two.matched, -1.0, int(two.matched.kf_id[0]), two.matched.kf_pose[1])
    atlas.close(two.matched, two.matched_database, reset=False, dated=True)
    atlas.start()
    atlas.date(active)
    for seconds, (reference, at) in enumerate(PLACED):
        atlas.place_frame(active, float(seconds), int(active.kf_id[reference]), active.kf_pose[at])
    atlas.record(culled)
    to_world = loops.merge_transform(culled, found)
    merged = loops.merge(
        culled, two.matched, CAMERA, found, mode="monocular", options=OPTIONS_NO_GLOBAL_BA
    )
    atlas.merge(0, merged, to_world)
    return _Merged(atlas, active, culled, merged, to_world, atlas.trajectory(merged.map))


@pytest.fixture(scope="module")
def merged(two_scaled: TwoMaps) -> _Merged:
    """Merge as :func:`_merge` does, no frame tracked in the matched map."""
    return _merge(two_scaled)


def test_a_frame_of_the_merged_away_map_is_placed_in_the_matched_maps_units(
    two_scaled: TwoMaps, merged: _Merged
) -> None:
    """Each frame lands where it was in truth, through its reference keyframe or a culled one."""
    for row, (_, at) in enumerate(PLACED):
        placed = maps.camera_centre(merged.trajectory.poses[row])
        true = maps.camera_centre(two_scaled.poses[SPLIT + at])
        assert np.linalg.norm(placed - true) < PLACE_BOUND
    assert merged.trajectory.maps.tolist() == [0, 0]
    assert merged.trajectory.tracked_maps.tolist() == [1, 1]


def test_what_the_active_map_removed_is_carried_into_the_matched_world(merged: _Merged) -> None:
    """The removed points carried by ``G``, the culled keyframe by ``rigid_of(T G⁻¹)``, exactly."""
    life = merged.atlas.lifetimes(merged.merged.map)
    culled = merged.culled
    points = merged.atlas.point_run_ids(culled.gone_pt_id + merged.merged.point_offset)
    expected = culled.gone_pt_xyz @ merged.to_world[:3, :3].T + merged.to_world[:3, 3]
    assert_close(life.points[points], expected)
    keyframe = int(merged.atlas.run_ids(culled.gone_kf_id + merged.merged.keyframe_offset)[0])
    back = sim3.inverse(merged.to_world)
    pose = loops.rigid_of(merged.active.kf_pose[CULLED] @ back)
    assert_close(life.keyframe_poses[keyframe], pose)
    assert life.keyframe_maps[keyframe] == 0


def test_a_kept_map_that_is_not_there_is_refused_by_its_number(merged: _Merged) -> None:
    """Taking up, or merging with, a map the atlas does not keep: InvalidInputError naming it."""
    atlas = Atlas(OPTIONS)
    atlas.start()
    with pytest.raises(InvalidInputError, match="resume: map 7 is not kept"):
        atlas.resume(7)
    with pytest.raises(InvalidInputError, match="merge: map 7 is not kept"):
        atlas.merge(7, merged.merged, merged.to_world)


def test_a_merged_map_reset_drops_its_frames_and_those_merged_into_it(two_scaled: TwoMaps) -> None:
    """A merged map reset: the frames tracked in it and in the map merged into it all go.

    Each of them ends in the merged map, and a reset map's frames are not in the trajectory (the
    run format). B4's review panel, run 2, A12: the reset dropped only the frames tracked in the
    map by its number, and kept the merged-away map's.
    """
    merged = _merge(two_scaled, in_matched=True)
    assert merged.trajectory.maps.tolist() == [0, 0, 0]
    assert merged.trajectory.tracked_maps.tolist() == [0, 1, 1]
    merged.atlas.close(merged.merged.map, None, reset=True, dated=False)
    assert len(merged.atlas.trajectory(None).timestamps) == 0
