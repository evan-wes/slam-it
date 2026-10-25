"""The keyframe database's tests: its index, covisibility, and relocalisation's candidates.

On the room's truth map, its keyframes' words those of the committed vocabulary; loop closing's
candidates on the loop room's map too, whose keyframes far apart share no point.
"""

from __future__ import annotations

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import covisibility, database, frames
from slam_it.edge import map as maps
from slam_it.edge.database import Database, Indexed
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations import vocabulary
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import KEYFRAMES, Truth, TwoMaps
from tests.edge.support import OPTIONS


def _frame_vector(
    truth: Truth, words: vocabulary.Vocabulary, index: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return one of the room's frames' word vector."""
    found, _ = vocabulary.quantize(words, truth.frames[index].features.descriptors)
    return vocabulary.bow(words, found)


def _assert_same_candidates(got: database.Candidates, expected: database.Candidates) -> None:
    """Assert two searches found the same candidates in the same order, scores and counts too."""
    np.testing.assert_array_equal(got.keyframes, expected.keyframes)
    np.testing.assert_array_equal(got.scores, expected.scores)
    assert (got.sharing, got.scored) == (expected.sharing, expected.scored)


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
    """A candidate stands for its group: its score is its own and its scored links', summed.

    With ``relocalization.common_words`` lowered, every keyframe sharing a word is scored, so the
    best candidate's links are too, and its group's score is more than its own.
    """
    vector = _frame_vector(truth, words, 25)
    every = OPTIONS.with_settings({"relocalization.common_words": 0.1})
    found = database.candidates(room_database, truth.map, vector, every)
    assert found.scored >= 2
    own = vocabulary.score(vector, room_database.vector(int(found.keyframes[0])))
    assert found.scores[0] > own


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
    ids=["defaults", "best link only", "looser"],
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
    _assert_same_candidates(got, expected)


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


# --- loop closing's candidates ------------------------------------------------------------

BEST_LINK_ONLY = OPTIONS.with_settings({"covisibility.min_points": 1 << 20})
"""Each keyframe linked to the one it shares most with alone: groups of one."""

LoopKeyframes = tuple[list[frames.Frame], list[np.ndarray], Map]


def _first_sharing(m: Map, n: int) -> Map:
    """Return the room's truth map, its first keyframe seeing only `n` of the points the last sees.

    Its other observations of them forgotten; the last keyframe's links are the two between,
    which share 61 and 100 points with it, more than ``covisibility.min_points`` (15).
    """
    seen = m.kp_point[m.keypoint_rows(m.keyframes - 1)]
    first = np.arange(m.kf_start[0], m.kf_start[1])
    shared = first[np.isin(m.kp_point[first], seen[seen >= 0])]
    return maps.forget_observations(m, shared[n:])


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_keyframe_sharing_a_point_is_never_a_loop_candidate(
    backend: str, truth: Truth, room_database: Database
) -> None:
    """A keyframe sharing 1 to 14 points, so not linked, is never a candidate; sharing none, it is.

    The truth map's first keyframe and its last (ORB-SLAM3's ``GetConnectedKeyFrames``).
    """
    chosen = require_function(backend, database.__name__, "loop_candidates")
    last = truth.map.keyframes - 1
    for n in range(15):
        m = _first_sharing(truth.map, n)
        rows, weights = covisibility.connected(m, last)
        assert dict(zip(rows.tolist(), weights.tolist(), strict=True)).get(0, 0) == n
        assert covisibility.links(m, last, OPTIONS)[0].tolist() == [2, 1]
        found = database.loop_candidates(
            room_database, m, int(m.kf_id[last]), OPTIONS, backend=chosen
        )
        assert found.keyframes.tolist() == ([0] if n == 0 else []), f"sharing {n}"


@pytest.mark.parametrize("settings", [OPTIONS, BEST_LINK_ONLY], ids=["defaults", "best link only"])
def test_a_keyframe_s_loop_candidates_share_no_point_with_it(
    loop_keyframes: LoopKeyframes, loop_database: Database, settings: Options
) -> None:
    """Never the keyframe or one sharing a point with it; at most ``loops.candidates``, best first.

    On the loop room's map, where every keyframe has candidates.
    """
    m = loop_keyframes[2]
    for k in range(m.keyframes):
        keyframe = int(m.kf_id[k])
        sharing = set(m.kf_id[covisibility.connected(m, k)[0]].tolist())
        found = database.loop_candidates(loop_database, m, keyframe, settings)
        assert len(found.keyframes), f"the loop room's keyframe {keyframe} has candidates"
        assert keyframe not in found.keyframes
        assert not sharing & set(found.keyframes.tolist())
        assert len(found.keyframes) <= settings.loops.candidates
        assert np.all(np.diff(found.scores) <= 0.0)
        assert found.sharing <= m.keyframes - 1 - len(sharing)
        assert found.scored <= found.sharing


def test_only_the_best_groups_are_kept(
    loop_keyframes: LoopKeyframes, loop_database: Database
) -> None:
    """With one candidate asked, the best group alone comes back, the first of three asked."""
    m = loop_keyframes[2]
    looser = BEST_LINK_ONLY.with_settings({"loops.common_words": 0.1})
    keyframe = int(m.kf_id[-1])
    three = database.loop_candidates(loop_database, m, keyframe, looser)
    one = database.loop_candidates(
        loop_database, m, keyframe, looser.with_settings({"loops.candidates": 1})
    )
    assert len(three.keyframes) == 3
    assert one.keyframes.tolist() == three.keyframes[:1].tolist()
    assert (one.sharing, one.scored) == (three.sharing, three.scored)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    "settings",
    [
        OPTIONS,
        BEST_LINK_ONLY,
        BEST_LINK_ONLY.with_settings(
            {"loops.common_words": 0.5, "loops.neighbours": 1, "loops.candidates": 2}
        ),
    ],
    ids=["defaults", "best link only", "looser"],
)
def test_every_backend_finds_python_s_loop_candidates(
    backend: str, settings: Options, loop_keyframes: LoopKeyframes, loop_database: Database
) -> None:
    """The same candidates in the same order, the same sums of the same scores: exactly."""
    chosen = require_function(backend, database.__name__, "loop_candidates")
    m = loop_keyframes[2]
    for keyframe in m.kf_id.tolist():
        expected = database.loop_candidates(loop_database, m, keyframe, settings)
        got = database.loop_candidates(loop_database, m, keyframe, settings, backend=chosen)
        _assert_same_candidates(got, expected)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_a_keyframe_outside_the_map_or_the_database_is_refused(
    backend: str, truth: Truth, words: vocabulary.Vocabulary, room_database: Database
) -> None:
    """Python refuses either before any backend runs, and each core refuses it too."""
    chosen = require_function(backend, database.__name__, "loop_candidates")
    compiled = backends.implementation(chosen, database.__name__)
    m = truth.map
    with pytest.raises(InvalidInputError, match="not in the map"):
        database.loop_candidates(room_database, m, m.next_kf_id + 2, OPTIONS)
    with pytest.raises(InvalidInputError, match="not in it"):
        database.loop_candidates(Database(words), m, int(m.kf_id[0]), OPTIONS)
    for db, keyframe in ((room_database, m.next_kf_id + 2), (Database(words), int(m.kf_id[0]))):
        with pytest.raises(InvalidInputError, match="not in"):
            compiled.loop_candidates(db.as_dict(), m.as_dict(), keyframe, OPTIONS.as_dict())


# --- a merge's candidates (milestone 2B's B4) -----------------------------------------------


def _matched_map(two: TwoMaps) -> list[Indexed]:
    """Return the two maps' matched map, as a merge's candidates are looked for in it."""
    return [Indexed(0, two.matched, two.matched_database)]


@pytest.mark.parametrize("variant", ["rigid", "scaled"])
def test_a_merge_s_candidates_are_where_the_path_comes_back(
    variant: str, request: pytest.FixtureRequest
) -> None:
    """The second last active keyframe's candidates are the first map's start, scored there.

    Of the first map's keyframes sharing a word with it, only one shares 80% of the most words
    any shares, so one candidate is scored.
    """
    two: TwoMaps = request.getfixturevalue(f"two_{variant}")
    keyframe = int(two.active.kf_id[-2])
    found = database.merge_candidates(
        two.active_database.vector(keyframe), _matched_map(two), OPTIONS
    )
    assert 1 <= len(found.keyframes) <= OPTIONS.loops.candidates
    np.testing.assert_array_equal(found.maps, [0] * len(found.keyframes))
    assert found.keyframes[0] <= 4
    assert np.all(np.diff(found.scores) <= 0.0)
    assert found.scored <= found.sharing


def test_each_map_s_words_are_counted_within_it(two_rigid: TwoMaps) -> None:
    """The same map twice, under two numbers: each gives its own best, ranked together.

    Counted over both, the most words any keyframe shares would be the same, and the cut too;
    counted within each, every map gives its candidates whatever another holds.
    """
    keyframe = int(two_rigid.active.kf_id[-2])
    vector = two_rigid.active_database.vector(keyframe)
    once = database.merge_candidates(vector, _matched_map(two_rigid), OPTIONS)
    copy = Indexed(3, two_rigid.matched, two_rigid.matched_database)
    twice = database.merge_candidates(vector, [*_matched_map(two_rigid), copy], OPTIONS)
    assert twice.sharing == 2 * once.sharing
    assert twice.scored == 2 * once.scored
    assert set(twice.maps.tolist()) == {0, 3}
    assert twice.scores[0] == twice.scores[1] == once.scores[0]
    np.testing.assert_array_equal(twice.keyframes[:2], [once.keyframes[0]] * 2)
    with pytest.raises(InvalidInputError, match="each map once"):
        database.merge_candidates(vector, _matched_map(two_rigid) * 2, OPTIONS)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_every_backend_finds_a_merge_s_candidates_as_python_does(
    backend: str, two_rigid: TwoMaps
) -> None:
    """Each active keyframe's candidates in the matched map and its copy: exactly Python's."""
    chosen = require_function(backend, database.__name__, "merge_candidates")
    copy = Indexed(3, two_rigid.matched, two_rigid.matched_database)
    others = [*_matched_map(two_rigid), copy]
    for keyframe in two_rigid.active.kf_id.tolist():
        vector = two_rigid.active_database.vector(keyframe)
        expected, got = (
            database.merge_candidates(vector, others, OPTIONS, backend=which)
            for which in ("python", chosen)
        )
        np.testing.assert_array_equal(got.maps, expected.maps)
        np.testing.assert_array_equal(got.keyframes, expected.keyframes)
        np.testing.assert_array_equal(got.scores, expected.scores)
        assert (got.sharing, got.scored) == (expected.sharing, expected.scored)
