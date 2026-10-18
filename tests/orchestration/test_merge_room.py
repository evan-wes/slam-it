"""The merge room's frames: the loop room blinded until a lost map is kept and a new one starts.

The frames are :func:`tests.orchestration.datasets.merge_frames`'.

Its bar, every language merging the new map into the first, comes with merging in the pipeline
(milestone 2B's B4, its second phase); here only the frames are held to what that bar needs.
"""

from __future__ import annotations

import numpy as np

from slam_it.edge.options import Options
from tests.edge import room
from tests.orchestration.datasets import MERGE_AT, MERGE_BLANKS, merge_frames


def _frames(count: int) -> list[room.Frame]:
    """Return tiny frames at the room's 30 Hz, each lit."""
    lit = np.full((2, 3), 9, dtype=np.uint8)
    depth = np.ones((2, 3))
    return [room.Frame(k / 30.0, lit, depth, lit, np.eye(4)) for k in range(count)]


def test_the_merge_room_is_blind_for_longer_than_the_loss_window() -> None:
    """Its black frames outlast ``loss.relocalize_seconds``, and end before the loop comes back."""
    given = _frames(room.LOOP_FRAMES)
    made = merge_frames(given)
    assert len(made) == len(given)
    black = [k for k, f in enumerate(made) if not np.any(f.gray)]
    assert black == list(range(MERGE_AT, MERGE_AT + MERGE_BLANKS))
    for k in black:
        assert not np.any(made[k].depth)
        assert made[k].right is not None
        assert not np.any(made[k].right)
    assert [f.timestamp for f in made] == [f.timestamp for f in given]
    seconds = made[black[-1] + 1].timestamp - made[black[0]].timestamp
    assert seconds > Options().loss.relocalize_seconds
    back = room.LOOP_FRAMES * 2.0 * np.pi / room.LOOP_TURN  # the frame at 360 degrees
    assert black[-1] < back
