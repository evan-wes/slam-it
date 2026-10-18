"""A vocabulary's folder: the files that keep place recognition's tree, written and read.

The package also holds the committed vocabulary (``default/``, :data:`DEFAULT`) and the record of
the vocabularies kept outside the repository (``local.json``), beside the code that reads them.

The tree and what it computes are rung 0's (:class:`slam_it.foundations.vocabulary.Vocabulary`);
the algorithm rungs take a vocabulary, never its files (the ladder: arrays, never files). Its
folder is read here, and by the native eval libraries' twins (``slam_it_eval``,
``slam-it-eval``), with which the native programs read it. A folder holds:

- ``descriptors.npy``, ``(N, 32)`` uint8: each node's centre (the root's zeros);
- ``parents.npy``, ``(N,)`` int32: each node's parent, -1 for the root;
- ``children.npy``, ``(N, 2)`` int32: each node's first child and child count (a node's
  children are consecutive; a leaf has none);
- ``words.npy``, ``(N,)`` int32: a leaf's word (words numbered in node order), -1 for an inner
  node;
- ``weights.npy``, ``(W,)`` float64: each word's weight;
- ``header.npy``, ``(5,)`` int64: the folder's format version (:data:`VERSION`), the version of
  the descriptor its words were trained on (:data:`slam_it.foundations.orb.DESCRIPTOR_VERSION`),
  k, levels and the number of words (:data:`HEADER` names them);
- ``sha256.npy``, ``(5, 32)`` uint8: the SHA-256 of each of the five arrays' files, in
  :data:`slam_it.foundations.vocabulary.ARRAYS`' order;
- ``training.json``: what training recorded of its making (the training set's name, its images
  and descriptors, the seed), for whoever reads the folder; nothing checks it.

Every language's reader refuses a folder of another format version, or one trained on another
descriptor (its words would mean nothing to this ORB), and checks the tree. The checksums are
Python's alone (:func:`read`), the ladder's one exception to its "both languages fail the same
way": a native program is launched by :func:`slam_it.orchestration.vo.run`, which reads the
folder first, and one run on its own trusts its arrays, as no SHA-256 is written natively. What
a run with a keyframe database (relocalising or closing loops) records of its vocabulary
(:func:`record`) is its header and checksums, which every language reads, so the three write
the same record.
"""

from __future__ import annotations

import functools
import hashlib
import io
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from slam_it.foundations.errors import InvalidInputError
from slam_it.foundations.orb import DESCRIPTOR_VERSION
from slam_it.foundations.vocabulary import ARRAYS, Vocabulary

VERSION = 2
"""The folder's format version, ``header.npy``'s first number (2: the header and checksums in
``.npy`` arrays, which the native readers check; 1 kept them in a ``vocabulary.json``)."""

HEADER = ("version", "descriptor_version", "k", "levels", "words")
"""``header.npy``'s numbers, in order."""

SHA256_BYTES = 32
"""A SHA-256 digest's bytes."""

TRAINING = "training.json"
"""The file of what training recorded."""

SHAPES: dict[str, tuple[type[np.generic], tuple[int, ...]]] = {
    "descriptors": (np.uint8, (-1, -1)),
    "parents": (np.int32, (-1,)),
    "children": (np.int32, (-1, 2)),
    "words": (np.int32, (-1,)),
    "weights": (np.float64, (-1,)),
}
"""Each array's type and shape as read (a -1 any length); the tree's own check does the rest."""

DEFAULT = Path(__file__).parent / "default"
"""The committed vocabulary, every run's that keeps a keyframe database unless it names another:
5 levels of k = 10, trained by ``scripts/train_vocabulary.py`` on the 9,969 of 10,000 frames
drawn from 20 ARKitScenes captures of its training fold that yielded descriptors, none from a
visit the test set films (``docs/design/2026-10-14-online-slam.md``)."""


def _npy(array: NDArray[Any]) -> bytes:
    """Return an array as the bytes of a ``.npy`` file."""
    buffer = io.BytesIO()
    np.save(buffer, array, allow_pickle=False)
    return buffer.getvalue()


def _replace(path: Path, data: bytes) -> None:
    """Write a file whole: to a temporary sibling, then renamed over it."""
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(data)
    temporary.replace(path)


def write(folder: str | Path, vocabulary: Vocabulary) -> None:
    """Write a vocabulary's folder (the module docstring), each file whole.

    Parameters
    ----------
    folder : str or pathlib.Path
        The folder: created if needed; its files are replaced.
    vocabulary : slam_it.foundations.vocabulary.Vocabulary
        The vocabulary; its ``info`` is written as ``training.json``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If its ``info`` cannot be written as JSON (nothing is written then).
    """
    try:
        training = json.dumps(vocabulary.info, indent=1, allow_nan=False) + "\n"
    except (TypeError, ValueError) as error:
        raise InvalidInputError(f"info: not JSON ({error})") from None
    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    sums = []
    for name in ARRAYS:
        data = _npy(getattr(vocabulary, name))
        sums.append(np.frombuffer(hashlib.sha256(data).digest(), dtype=np.uint8))
        _replace(out / f"{name}.npy", data)
    header = [VERSION, DESCRIPTOR_VERSION, vocabulary.k, vocabulary.levels, vocabulary.size]
    _replace(out / "header.npy", _npy(np.array(header, dtype=np.int64)))
    _replace(out / "sha256.npy", _npy(np.stack(sums)))
    _replace(out / TRAINING, training.encode())


def _load(path: Path, data: bytes, dtype: type[np.generic], shape: tuple[int, ...]) -> NDArray[Any]:
    """Return a ``.npy`` file's array, of `dtype` and `shape` (a -1 any length on that axis).

    The header is read first, and the data must fill its shape exactly, so that a shape too
    large is refused before anything is allocated; a Fortran-order array is refused, as the
    native readers refuse it. The array is a read-only view of `data`.
    """
    buffer = io.BytesIO(data)
    try:
        major, _ = np.lib.format.read_magic(buffer)
        read_header = (
            np.lib.format.read_array_header_1_0
            if major == 1
            else np.lib.format.read_array_header_2_0
        )
        found, fortran, found_dtype = read_header(buffer)
    except (ValueError, EOFError) as error:  # not an .npy file, or a malformed header
        raise InvalidInputError(f"{path}: not a readable .npy array ({error})") from None
    if fortran:
        raise InvalidInputError(f"{path}: expected a C-order .npy array")
    if (
        found_dtype != dtype
        or len(found) != len(shape)
        or any(s not in (-1, n) for s, n in zip(shape, found, strict=True))
    ):
        raise InvalidInputError(
            f"{path}: expected {np.dtype(dtype).name} {shape}, got {found_dtype} {found}"
        )
    count = math.prod(found)
    if count * found_dtype.itemsize != len(data) - buffer.tell():
        raise InvalidInputError(f"{path}: its data do not fill its shape")
    return np.frombuffer(data, dtype, count=count, offset=buffer.tell()).reshape(found)


def _read_header(root: Path) -> tuple[dict[str, int], NDArray[np.uint8]]:
    """Read a folder's header, by :data:`HEADER`'s names, and its checksums.

    This format's version, this ORB's descriptor.
    """
    path = root / "header.npy"
    numbers = _load(path, path.read_bytes(), np.int64, (len(HEADER),))
    header = dict(zip(HEADER, numbers.tolist(), strict=True))
    version, descriptor = header["version"], header["descriptor_version"]
    if version != VERSION:
        raise InvalidInputError(
            f"{root}: a vocabulary of format version {version}, expected {VERSION}"
        )
    if descriptor != DESCRIPTOR_VERSION:
        raise InvalidInputError(
            f"{root}: trained on descriptor version {descriptor}, this ORB's is "
            f"{DESCRIPTOR_VERSION}: its words mean nothing here"
        )
    path = root / "sha256.npy"
    sums = _load(path, path.read_bytes(), np.uint8, (len(ARRAYS), SHA256_BYTES))
    return header, sums


def record(folder: str | Path) -> dict[str, Any]:
    """Return what a run with a keyframe database records of its vocabulary: header, checksums.

    Parameters
    ----------
    folder : str or pathlib.Path
        The vocabulary's folder (its arrays are not read: :func:`read` checks them).

    Returns
    -------
    dict
        :data:`HEADER`'s numbers, and ``sha256``: each array's SHA-256, in hex, by name.

    Raises
    ------
    FileNotFoundError
        If the header or checksums are missing.
    slam_it.foundations.errors.InvalidInputError
        If the header is malformed, of another format version or another descriptor's, or the
        checksums are malformed.
    """
    header, sums = _read_header(Path(folder))
    return header | {
        "sha256": {name: bytes(row).hex() for name, row in zip(ARRAYS, sums, strict=True)}
    }


def check_record(value: object) -> None:
    """Check a run's record of its vocabulary, as :func:`record` makes it.

    Parameters
    ----------
    value : object
        The record.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not an object of :data:`HEADER`'s integers and ``sha256``, an object of each
        array's 64 hex digits, with no other key.
    """
    expected = "expected an object of its header's integers and each array's SHA-256"
    if not isinstance(value, dict) or set(value) != {*HEADER, "sha256"}:
        raise InvalidInputError(f"vocabulary: {expected}")
    if not all(type(value[key]) is int for key in HEADER):
        raise InvalidInputError(f"vocabulary: {expected}")
    sums = value["sha256"]
    if not isinstance(sums, dict) or set(sums) != set(ARRAYS):
        raise InvalidInputError(f"vocabulary: {expected}")
    for digest in sums.values():
        if not (
            isinstance(digest, str)
            and len(digest) == 2 * SHA256_BYTES
            and all(c in "0123456789abcdef" for c in digest)
        ):
            raise InvalidInputError(f"vocabulary: {expected}")


def read(folder: str | Path) -> Vocabulary:
    """Read a vocabulary's folder, every array checked against its recorded SHA-256.

    Parameters
    ----------
    folder : str or pathlib.Path
        The folder :func:`write` wrote.

    Returns
    -------
    slam_it.foundations.vocabulary.Vocabulary
        The vocabulary, its ``info`` its ``training.json``, its arrays read-only views of the
        files' bytes.

    Raises
    ------
    FileNotFoundError
        If a file is missing.
    slam_it.foundations.errors.InvalidInputError
        If the header is not this format version's, the vocabulary was trained on another
        descriptor (:data:`slam_it.foundations.orb.DESCRIPTOR_VERSION`), an array's SHA-256
        differs from the recorded one, an array or the tree is malformed, the header's words
        are not the tree's, or ``training.json`` is not a JSON object.
    """
    root = Path(folder)
    header, sums = _read_header(root)
    arrays = {}
    for name, digest in zip(ARRAYS, sums, strict=True):
        path = root / f"{name}.npy"
        data = path.read_bytes()
        if hashlib.sha256(data).digest() != bytes(digest):
            raise InvalidInputError(f"{path}: its SHA-256 is not the recorded one")
        arrays[name] = _load(path, data, *SHAPES[name])
    path = root / TRAINING
    try:
        info = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise InvalidInputError(f"{path}: not JSON ({error})") from None
    if not isinstance(info, dict):
        raise InvalidInputError(f"{path}: expected a JSON object")
    words = header["words"]
    out = Vocabulary(
        header["k"], header["levels"], arrays["descriptors"], arrays["parents"],
        arrays["children"], arrays["words"], arrays["weights"], info,
    )  # fmt: skip
    if out.size != words:
        raise InvalidInputError(f"{root}: the header counts {words} words, the tree {out.size}")
    return out


@functools.cache
def default() -> Vocabulary:
    """Return the committed vocabulary (:data:`DEFAULT`), read once.

    Returns
    -------
    slam_it.foundations.vocabulary.Vocabulary
        The vocabulary, shared by every caller: its arrays are read-only (:func:`read`'s).
    """
    return read(DEFAULT)
