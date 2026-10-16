"""The keyframe database's tests: its index, covisibility, and relocalisation's candidates.

On the room's truth map, its keyframes' words those of the committed vocabulary.
"""

from __future__ import annotations

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import database
from slam_it.edge.database import Database
from slam_it.edge.options import Options
from slam_it.foundations import vocabulary
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import KEYFRAMES, Truth
from tests.edge.support import OPTIONS


def _frame_vector(
    truth: Truth, words: vocabulary.Vocabulary, index: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return one of the room's frames' word vector."""
    found, _ = vocabulary.quantize(words, truth.frames[index].features.descriptors)
    return vocabulary.bow(words, found)


def test_the_index_holds_each_keyframe_under_its_words(room_database: Database) -> None:
    """Every word of a keyframe's vector lists it, the lists ascending; the vectors sum to 1."""
    for keyframe in room_database.keyframes.tolist():
        ids, values = room_database.vector(keyframe)
        assert np.isclose(values.sum(), 1.0)
        for word in ids.tolist():
            holders = room_database.holders(word)
            assert keyframe in holders
            assert holders == sorted(holders)


def test_a_culled_keyframe_leaves_the_index(truth: Truth, words: vocabulary.Vocabulary) -> None:
    """Erased, a keyframe is under none of its words; erasing it again, or adding twice, fails."""
    db = Database(words)
    m = truth.map
    for k in range(m.keyframes):
        db.add(int(m.kf_id[k]), m.kp_descriptor[m.keypoint_rows(k)])
    with pytest.raises(InvalidInputError, match="keyframe 1 is already in it"):
        db.add(1, m.kp_descriptor[m.keypoint_rows(1)])
    ids, _ = db.vector(1)
    db.erase(1)
    assert db.keyframes.tolist() == [0, 2, 3]
    assert all(1 not in db.holders(word) for word in ids.tolist())
    with pytest.raises(InvalidInputError, match="keyframe 1 is not in it"):
        db.erase(1)


def test_the_database_crosses_keyframe_by_keyframe(room_database: Database) -> None:
    """as_dict lays each keyframe's vector and nodes end to end, by id."""
    crossed = room_database.as_dict()
    np.testing.assert_array_equal(crossed["kf_id"], room_database.keyframes)
    for i, keyframe in enumerate(crossed["kf_id"].tolist()):
        words = slice(crossed["bow_start"][i], crossed["bow_start"][i + 1])
        ids, values = room_database.vector(keyframe)
        np.testing.assert_array_equal(crossed["bow_word"][words], ids)
        np.testing.assert_array_equal(crossed["bow_value"][words], values)
        nodes = slice(crossed["node_start"][i], crossed["node_start"][i + 1])
        np.testing.assert_array_equal(crossed["node"][nodes], room_database.nodes(keyframe))


def test_covisible_keyframes_share_the_most_points(truth: Truth) -> None:
    """Each keyframe's best covisible keyframes share at least covisible_points, most first."""
    m = truth.map
    keyframes = m.kf_id.tolist()
    best = database.best_covisible(m, keyframes, OPTIONS)
    seen = {
        k: set(m.kp_point[m.keypoint_rows(row)][m.kp_point[m.keypoint_rows(row)] >= 0].tolist())
        for row, k in enumerate(keyframes)
    }
    for k in keyframes:
        shared = {other: len(seen[k] & seen[other]) for other in keyframes if other != k}
        expected = sorted(
            (o for o, n in shared.items() if n >= OPTIONS.covisibility.min_points),
            key=lambda o: (-shared[o], o),
        )
        assert best[k] == expected[: OPTIONS.relocalization.neighbours]
    assert any(best[k] for k in keyframes)
    none = database.best_covisible(
        m, keyframes, OPTIONS.with_settings({"covisibility.min_points": 1 << 20})
    )
    assert all(not neighbours for neighbours in none.values())


@pytest.mark.parametrize(("frame", "keyframe"), [(1, 0), (13, 1), (25, 2), (37, 3)])
def test_a_frame_of_the_room_finds_the_keyframe_beside_it(
    truth: Truth,
    words: vocabulary.Vocabulary,
    room_database: Database,
    frame: int,
    keyframe: int,
) -> None:
    """The keyframe a frame after it ranks first; the scores fall, and every keyframe shares."""
    assert KEYFRAMES[keyframe] == frame - 1
    found = database.candidates(
        room_database, truth.map, _frame_vector(truth, words, frame), OPTIONS
    )
    assert found.keyframes[0] == keyframe
    assert np.all(np.diff(found.scores) <= 0.0)
    assert found.sharing == truth.map.keyframes
    assert 1 <= found.scored <= found.sharing


def test_a_group_adds_its_scored_neighbours(
    truth: Truth, words: vocabulary.Vocabulary, room_database: Database
) -> None:
    """Without covisibility a candidate's score is its own; with it, its group's, no less."""
    vector = _frame_vector(truth, words, 25)
    alone = database.candidates(
        room_database,
        truth.map,
        vector,
        OPTIONS.with_settings({"covisibility.min_points": 1 << 20}),
    )
    for keyframe, score in zip(alone.keyframes.tolist(), alone.scores.tolist(), strict=True):
        assert score == vocabulary.score(vector, room_database.vector(keyframe))
    grouped = database.candidates(room_database, truth.map, vector, OPTIONS)
    assert grouped.scores[0] >= alone.scores[0]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("frame", [1, 13, 25, 37])
@pytest.mark.parametrize(
    "settings",
    [
        OPTIONS,
        OPTIONS.with_settings({"covisibility.min_points": 1 << 20}),
        OPTIONS.with_settings(
            {
                "relocalization.common_words": 0.5,
                "relocalization.group_score": 0.5,
                "relocalization.neighbours": 1,
            }
        ),
    ],
    ids=["defaults", "no covisibility", "looser"],
)
def test_every_backend_finds_python_s_candidates(  # noqa: PLR0917 -- cases, fixtures
    backend: str,
    frame: int,
    settings: Options,
    truth: Truth,
    words: vocabulary.Vocabulary,
    room_database: Database,
) -> None:
    """The same candidates in the same order, the same sums of the same scores: exactly."""
    chosen = require_function(backend, database.__name__, "candidates")
    vector = _frame_vector(truth, words, frame)
    expected = database.candidates(room_database, truth.map, vector, settings)
    got = database.candidates(room_database, truth.map, vector, settings, backend=chosen)
    np.testing.assert_array_equal(got.keyframes, expected.keyframes)
    np.testing.assert_array_equal(got.scores, expected.scores)
    assert (got.sharing, got.scored) == (expected.sharing, expected.scored)


def test_a_frame_sharing_no_word_has_no_candidates(truth: Truth, room_database: Database) -> None:
    """An empty vector shares nothing: no keyframe is weighed."""
    empty = (np.zeros(0, dtype=np.int64), np.zeros(0))
    found = database.candidates(room_database, truth.map, empty, OPTIONS)
    assert (found.sharing, found.scored, len(found.keyframes)) == (0, 0, 0)


def test_a_malformed_vector_is_refused(truth: Truth, room_database: Database) -> None:
    """Words that do not ascend, values not one per word, or not integers: InvalidInputError.

    The vocabulary's check (:func:`slam_it.foundations.vocabulary.as_vector`).
    """
    for ids, values in (([2, 1], [0.5, 0.5]), ([1, 2], [1.0]), ([1.9, 2.5], [0.5, 0.5])):
        with pytest.raises(InvalidInputError, match="vector: expected"):
            database.candidates(room_database, truth.map, (ids, values), OPTIONS)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize("change", ["missing", "extra"])
def test_a_database_with_a_key_missing_or_extra_is_refused(
    backend: str, change: str, truth: Truth, room_database: Database
) -> None:
    """The database crosses as exactly :meth:`Database.as_dict`'s keys (review panel finding A4).

    A key one side added and the other did not know is refused, in both languages alike.
    """
    chosen = require_function(backend, database.__name__, "candidates")
    compiled = backends.implementation(chosen, database.__name__)
    crossed = room_database.as_dict()
    if change == "missing":
        del crossed["node"]
    else:
        crossed["bogus"] = np.zeros(0, dtype=np.int64)
    ids, values = vocabulary.bow(room_database.vocabulary, [0])
    with pytest.raises(ValueError, match=r"node|bogus|exactly the keys"):
        compiled.candidates(crossed, truth.map.as_dict(), ids, values, OPTIONS.as_dict())
