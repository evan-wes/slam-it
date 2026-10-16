"""A vocabulary's folder: written, read, and refused, as every language's reader is held to it.

``tests/fixtures/eval/vocabulary_cases.json`` (``generate_vocabulary.py``) holds the cases the
C++ and Rust readers' tests run too.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slam_it.eval import vocabularies
from slam_it.foundations import orb, vocabulary
from slam_it.foundations.errors import InvalidInputError
from tests.fixtures.eval.generate_vocabulary import FOLDER, OUT, TREE

CASES: list[dict[str, Any]] = json.loads(OUT.read_text())["cases"]


def _copy(case: dict[str, Any], tmp_path: Path) -> Path:
    """Return a copy of the fixture's folder with the case's files replaced."""
    folder = tmp_path / "vocabulary"
    shutil.copytree(FOLDER, folder)
    for name, data in case["files"].items():
        (folder / name).write_bytes(bytes.fromhex(data))
    return folder


def test_the_fixture_folder_is_what_write_writes(tmp_path: Path) -> None:
    """The committed folder is what :func:`write` writes of its tree now, file for file."""
    vocabularies.write(tmp_path, TREE)
    names = sorted(p.name for p in FOLDER.iterdir())
    assert sorted(p.name for p in tmp_path.iterdir()) == names
    for name in names:
        assert (tmp_path / name).read_bytes() == (FOLDER / name).read_bytes(), name


def test_a_vocabulary_round_trips_through_its_folder(tmp_path: Path) -> None:
    """Every array, k, the levels and the training record survive, read-only."""
    vocabularies.write(tmp_path, TREE)
    back = vocabularies.read(tmp_path)
    for name in vocabulary.ARRAYS:
        np.testing.assert_array_equal(getattr(back, name), getattr(TREE, name))
        assert not getattr(back, name).flags.writeable
    assert (back.k, back.levels, back.info) == (TREE.k, TREE.levels, TREE.info)


def test_a_run_records_the_header_and_checksums() -> None:
    """The record is the header's numbers and each array's SHA-256, as the fixture holds it."""
    record = vocabularies.record(FOLDER)
    assert record == json.loads(OUT.read_text())["record"]
    assert (record["version"], record["descriptor_version"]) == (2, orb.DESCRIPTOR_VERSION)
    vocabularies.check_record(record)


@pytest.mark.parametrize("case", CASES, ids=[c["name"] for c in CASES])
def test_the_reader_reads_or_refuses_each_case(case: dict[str, Any], tmp_path: Path) -> None:
    """Python's reader refuses exactly the cases it should, with their messages."""
    folder = _copy(case, tmp_path)
    if "python" in case["refused_by"]:
        with pytest.raises(InvalidInputError, match=case["match"]):
            vocabularies.read(folder)
    else:
        assert vocabularies.read(folder).size == TREE.size


def test_a_missing_file_is_not_found(tmp_path: Path) -> None:
    """A folder without its checksums: FileNotFoundError, as for any file Python reads."""
    folder = _copy({"files": {}}, tmp_path)
    (folder / "sha256.npy").unlink()
    with pytest.raises(FileNotFoundError):
        vocabularies.read(folder)


@pytest.mark.parametrize(
    "change",
    [
        {"words": True},  # a number that is not an integer
        {"k": "2"},
        {"extra": 1},
        {"sha256": {"descriptors": "00"}},
        {"sha256": {name: "g" * 64 for name in vocabulary.ARRAYS}},
    ],
)
def test_a_malformed_record_is_refused(change: dict[str, Any]) -> None:
    """A run's record of its vocabulary: the header's integers and five 64-digit checksums."""
    record = vocabularies.record(FOLDER) | change
    with pytest.raises(InvalidInputError, match="vocabulary: expected an object"):
        vocabularies.check_record(record)


def test_training_info_that_is_not_json_is_refused_before_anything_is_written(
    tmp_path: Path,
) -> None:
    """A training record JSON cannot hold: InvalidInputError, the folder untouched."""
    bad = vocabulary.Vocabulary(
        TREE.k, TREE.levels, TREE.descriptors, TREE.parents, TREE.children, TREE.words,
        TREE.weights, {"set": {1, 2}},
    )  # fmt: skip
    with pytest.raises(InvalidInputError, match="info: not JSON"):
        vocabularies.write(tmp_path / "out", bad)
    assert not (tmp_path / "out").exists()


def test_the_default_vocabulary_reads_and_is_the_documented_one() -> None:
    """The committed vocabulary reads, its checksums hold, 5 levels of k = 10; read-only."""
    vocab = vocabularies.default()
    assert (vocab.k, vocab.levels) == (10, 5)
    assert vocab.size == 99_961
    assert vocab.info["name"] == "arkitscenes-training"
    assert vocab.info["images"] == 9_969
    assert not vocab.weights.flags.writeable
    assert vocabularies.default() is vocab
