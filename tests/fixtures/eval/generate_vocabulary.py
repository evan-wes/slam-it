"""Generate the vocabulary reader's fixture: a small folder, ``vocabulary/``, and its cases.

Run from the repository root after changing the folder or the cases (the fixtures are
committed)::

    python -m tests.fixtures.eval.generate_vocabulary

Python's :func:`slam_it.eval.vocabularies.write` writes ``vocabulary/`` from :data:`TREE`, a tree
of three levels, k = 2 (``tests/eval/test_vocabularies.py`` holds the folder to what ``write``
writes now). ``vocabulary_cases.json`` holds the tree as read (``tree``), what a run records of
it (``record``), and the reader's cases: each a copy of the folder with some of its files
replaced (``files``: a file's name and its bytes, in hex), the languages whose reader refuses it
with an ``InvalidInput`` (``refused_by``) and a part of each one's message (``match``). A case
that changes an array also rewrites ``sha256.npy`` for it, so that Python's reader, which checks
the checksums first, meets the fault itself; ``tampered`` does not, and only Python refuses it
(the native readers do not check the checksums: the module docstring's exception). The C++ and
Rust readers' tests (``cpp/tests/eval/test_vocabularies.cpp``, ``slam-it-eval``'s
``vocabularies`` tests) and Python's run every case.
"""

from __future__ import annotations

import hashlib
import io
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import vocabularies
from slam_it.foundations.vocabulary import ARRAYS, Vocabulary
from tests.fixtures.foundations._layout import dumps

HERE = Path(__file__).parent
FOLDER = HERE / "vocabulary"
OUT = HERE / "vocabulary_cases.json"
EVERY = ["python", "cpp", "rust"]


def _byte(value: int) -> NDArray[np.uint8]:
    """Return a descriptor whose first byte is `value`, the rest zero."""
    out = np.zeros(32, dtype=np.uint8)
    out[0] = value
    return out


TREE = Vocabulary(
    k=2,
    levels=3,
    descriptors=np.stack(
        [_byte(c) for c in (0x00, 0x00, 0xF0, 0x00, 0x03, 0xF0, 0xFF, 0xFF, 0xFE)]
    ),
    parents=np.array([-1, 0, 0, 1, 1, 2, 2, 6, 6], dtype=np.int32),
    children=np.array(
        [[1, 2], [3, 2], [5, 2], [0, 0], [0, 0], [0, 0], [7, 2], [0, 0], [0, 0]], dtype=np.int32
    ),
    words=np.array([-1, -1, -1, 0, 1, 2, -1, 3, 4], dtype=np.int32),
    weights=np.array([0.5, 1.25, 2.0, 2.75, 3.5]),
    info={"name": "the reader's fixture"},
)
"""Root 0; level 1: nodes 1 and 2; level 2: 3 and 4 under 1, 5 and 6 under 2; level 3: 7 and 8
under 6. Five words, at nodes 3, 4, 5, 7 and 8."""


def _npy(array: NDArray[Any]) -> bytes:
    """Return an array as a ``.npy`` file's bytes."""
    buffer = io.BytesIO()
    np.save(buffer, array, allow_pickle=False)
    return buffer.getvalue()


def _with_header(data: bytes, old: bytes, new: bytes) -> bytes:
    """Return a ``.npy`` file's bytes with part of its header replaced, its length kept."""
    length = int.from_bytes(data[8:10], "little")
    header = data[10 : 10 + length]
    changed = header.replace(old, new)
    grown = len(changed) - len(header)
    padding = header[:-1].rstrip(b" ")
    assert len(header) - 1 - len(padding) > grown, "not enough padding to grow into"
    changed = changed[: -1 - grown] + b"\n" if grown > 0 else changed
    return data[:10] + changed + data[10 + length :]


def _arrays(**changed: bytes) -> dict[str, bytes]:
    """Return changed arrays' files, with ``sha256.npy`` rewritten for them."""
    sums = []
    for name in ARRAYS:
        data = changed.get(f"{name}.npy", (FOLDER / f"{name}.npy").read_bytes())
        sums.append(np.frombuffer(hashlib.sha256(data).digest(), dtype=np.uint8))
    return {**changed, "sha256.npy": _npy(np.stack(sums))}


def _header(**values: int) -> dict[str, bytes]:
    """Return ``header.npy`` with some of its numbers changed."""
    header = np.load(FOLDER / "header.npy")
    for name, value in values.items():
        header[vocabularies.HEADER.index(name)] = value
    return {"header.npy": _npy(header)}


def _cases() -> list[dict[str, Any]]:
    """Return the reader's cases (the module docstring)."""
    parents, children = TREE.parents.copy(), TREE.children.copy()
    parents[5] = -7
    past = TREE.children.copy()
    past[1] = [np.iinfo(np.int32).max, 2]  # first + count overflows an int32
    leaf = TREE.children.copy()
    leaf[3] = [2_000_000_000, 0]  # a leaf: its first child means nothing
    weights = (FOLDER / "weights.npy").read_bytes()
    descriptors = (FOLDER / "descriptors.npy").read_bytes()
    cases: list[tuple[str, dict[str, bytes], list[str], str]] = [
        ("the folder itself", {}, [], ""),
        ("a leaf's first child out of range", _arrays(**{"children.npy": _npy(leaf)}), [], ""),
        ("a parent after its child", _arrays(**{"parents.npy": _npy(parents)}), EVERY,
         "parents: each node's parent must come before it"),
        ("children past the int32 range", _arrays(**{"children.npy": _npy(past)}), EVERY,
         "children: expected every node but the root once, in range"),
        ("a shape no size_t holds", _arrays(**{"parents.npy": _with_header(
            (FOLDER / "parents.npy").read_bytes(), b"(9,)", b"(99999999999999999999999,)")}),
         EVERY, "parents"),
        ("a shape whose size overflows", _arrays(**{"descriptors.npy": _with_header(
            descriptors, b"(9, 32)", b"(2305843009213693952, 32)")}), EVERY, "descriptors"),
        ("a Fortran-order array", _arrays(**{"children.npy": _npy(np.asfortranarray(children))}),
         EVERY, "children"),
        ("an array of another type",
         _arrays(**{"parents.npy": _npy(TREE.parents.astype(np.int64))}), EVERY, "parents"),
        ("a file that is not .npy", _arrays(**{"weights.npy": b"not an array\n"}), EVERY,
         "weights"),
        ("an array cut short", _arrays(**{"descriptors.npy": descriptors[:-5]}), EVERY,
         "descriptors"),
        ("another format version", _header(version=1), EVERY,
         "a vocabulary of format version 1, expected 2"),
        ("another descriptor's words", _header(descriptor_version=99), EVERY,
         "trained on descriptor version 99"),
        ("a header that miscounts the words", _header(words=6), EVERY,
         "the header counts 6 words, the tree 5"),
        ("a k below 2", _header(k=1), EVERY, "k: must be from 2 to 64, got 1"),
        ("a leaf deeper than the levels", _header(levels=2), EVERY,
         "levels: a leaf lies deeper than"),
        ("a header of another shape", {"header.npy": _npy(np.arange(4, dtype=np.int64))}, EVERY,
         "header"),
        ("checksums of another shape", {"sha256.npy": _npy(np.zeros((5, 31), dtype=np.uint8))},
         EVERY, "sha256"),
        ("an array changed under its checksum",
         {"weights.npy": _npy(np.load(io.BytesIO(weights)) + 1.0)}, ["python"],
         "SHA-256 is not the recorded one"),
        ("a training record that is not an object", {vocabularies.TRAINING: b"[]\n"}, ["python"],
         "expected a JSON object"),
    ]  # fmt: skip
    return [
        {"name": name, "files": {f: data.hex() for f, data in files.items()},
         "refused_by": refused, "match": match}
        for name, files, refused, match in cases
    ]  # fmt: skip


def main() -> None:
    """Write the folder, then the cases."""
    if FOLDER.exists():
        shutil.rmtree(FOLDER)
    vocabularies.write(FOLDER, TREE)
    tree = {
        "k": TREE.k,
        "levels": TREE.levels,
        **{name: getattr(TREE, name).tolist() for name in ARRAYS},
    }
    doc = {"tree": tree, "record": vocabularies.record(FOLDER), "cases": _cases()}
    OUT.write_text(dumps(doc))
    print(f"wrote {FOLDER} and {OUT}: {len(doc['cases'])} cases")


if __name__ == "__main__":
    main()
    json.loads(OUT.read_text())  # it reads back
