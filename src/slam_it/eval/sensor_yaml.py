"""The ``sensor.yaml`` files of ASL-style datasets (EuRoC, ZJU-SenseTime): settings and lists.

Both datasets describe each sensor in a small YAML file of ``key: value`` settings and
``key: [a, b, ...]`` lists, a ``#`` starting a comment, and lists sometimes nested one level in
a block (EuRoC's ``T_BS: data: [...]``, ZJU-SenseTime's ``intrinsic: camera: [...]``). These
read the few keys a dataset's reader needs with regular expressions rather than a YAML library:
the files are that flat, and a key a reader needs that is missing or malformed is refused by
name.
"""

from __future__ import annotations

import re
from pathlib import Path

from slam_it.foundations.errors import InvalidInputError


def read_text(path: Path) -> str:
    """Return a sensor.yaml's text without its comments (from ``#`` to a line's end).

    No value in these files holds a ``#``.
    """
    return re.sub(r"#[^\n]*", "", path.read_text())


def numbers(text: str, key: str, path: Path) -> list[float]:
    """Return the numbers of a ``key: [a, b, ...]`` list, its values split by commas or spaces.

    Parameters
    ----------
    text : str
        The file's text, its comments removed (:func:`read_text`), or a block of it.
    key : str
        The list's key; its first occurrence is read.
    path : pathlib.Path
        The file, for messages.

    Returns
    -------
    list of float
        The list's numbers.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the key is missing or a value is not a number.
    """
    found = re.search(rf"^\s*{key}:\s*\[([^\]]*)\]", text, flags=re.MULTILINE)
    if found is None:
        raise InvalidInputError(f"{path}: no `{key}: [...]` list")
    try:
        return [float(v) for v in re.split(r"[,\s]+", found.group(1).strip()) if v]
    except ValueError as error:
        raise InvalidInputError(f"{path}: `{key}` holds a non-number ({error})") from None


def setting(text: str, key: str, path: Path) -> str:
    """Return a ``key: value`` setting's value: its first word.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the key is missing.
    """
    found = re.search(rf"^\s*{key}:\s*(\S+)", text, flags=re.MULTILINE)
    if found is None:
        raise InvalidInputError(f"{path}: no `{key}` setting")
    return found.group(1)
