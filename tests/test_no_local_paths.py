"""Guard: nothing in the repository names a place on one machine, or its owner.

Datasets and runs are named by settings or relative to the repository, so no tracked file
needs a home folder, a session's scratch folder or link, the folders the project has lived in
on one machine, or anything from one person's own tools; and the owner is named by their public
handle alone. This scans the text of every file git would commit (this one excepted, since it
spells out the patterns).

The owner's full name, the folders the project lived in and the personal tools' folders are not
written here: each is held as the SHA-256 of each of its words, lowercased, and every run of
words in a file is compared with them.
"""

from __future__ import annotations

import functools
import hashlib
import re
import subprocess
from collections.abc import Mapping
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
THIS = Path(__file__).resolve().relative_to(REPO).as_posix()

FORBIDDEN = {
    "a home folder": r"/home/[\w.-]+|/Users/[\w.-]+|[A-Za-z]:\\Users\\[\w.-]+",
    "a folder under the home folder": r"~/projects|~/\.claude",
    "a session's scratch folder": r"/tmp/claude-\d+",
    "a link to a Claude Code session": r"claude\.ai/code/session",
    "a note kept outside the repository": r"\[\[[a-z0-9]+(?:-[a-z0-9]+)+\]\]",
    "a personal email address": r"[\w.+-]*(?i:@gmail\.com)",
}
PATTERN = re.compile("|".join(f"(?P<g{i}>{p})" for i, p in enumerate(FORBIDDEN.values())))
WHAT = list(FORBIDDEN)

# What the guard holds as digests, by what it reports: each spelling the SHA-256 of each of its
# words, lowercased, in order. Anything but a letter parts two words, so a spelling is found
# whatever joins them: an underscore, a hyphen, a slash, a dot, a space or a line break.
HIDDEN: dict[str, tuple[tuple[str, ...], ...]] = {
    "the owner's full name": (
        (
            "ae74f72d212fb9871302a2459aeaf7b20bc2f792e4852be648a7d4e63967d9b1",
            "2be4e33aa3e21f6ff7ad144f8fd58228a1cce3b50340b4805ff0c63480eec488",
        ),
    ),
    "a folder the project lived in": (
        (
            "e590d8e002b4908002fd31c750d0b43e93b9ed13ee1eeb8842501e96795f2752",
            "244210e48437b6556980a70249a99369934a352429034cef9d7bd253b3bf2c01",
        ),
        (
            "8e6a0ab2937027144d3c821bfbc92cc153ab0d161908dffe54149df13044096a",
            "2ad8a7049d7c5511ac254f5f51fe70a046ebd884729056f0fe57f5160d467153",
            "5ef5ef0364b6939c4ca61f34b393f7b368d1be8619647aaf83d5b395919ab629",
            "244210e48437b6556980a70249a99369934a352429034cef9d7bd253b3bf2c01",
        ),
        (
            "8e6a0ab2937027144d3c821bfbc92cc153ab0d161908dffe54149df13044096a",
            "2ad8a7049d7c5511ac254f5f51fe70a046ebd884729056f0fe57f5160d467153",
            "244210e48437b6556980a70249a99369934a352429034cef9d7bd253b3bf2c01",
        ),
        (
            "93b943cfeefedf0b64488bbe917d852a6ab9657c9e38955511bcdfcda5f1f9d9",
            "244210e48437b6556980a70249a99369934a352429034cef9d7bd253b3bf2c01",
        ),
    ),
    "a personal tool": (
        (
            "c857d09db23e6822e3600bc06ad8d58f92ed62bc8efd81c753f77048662cb97d",
            "8c5967fd8486f34493710cc39b240aad46536cf4ee421ffd0479e6542db03e36",
        ),
        ("556d1f14c80f008eb61334df7417e0adb53464a22552ce44913c435fd39f3fe5",),
    ),
}
_WORD = re.compile(r"[^\W\d_]+")

# Mentions kept on purpose: the Eigen-derived SVD carries Eigen's copyright lines as Eigen's
# files do, author addresses included (the MPL 2.0, section 3.4: the substance of a copyright
# notice may not be removed or altered); and the readiness PR's review report quotes, verbatim,
# a pattern of this guard's that its reviewer described.
ALLOWED = {
    ("rust/slam-it-foundations/src/linalg/jacobi_svd3.rs", "jacob.benoit.1@gmail.com"),
    ("docs/reviews/2026-10-18-chore-public-readiness.md", "~/projects"),
}


@functools.cache
def _digest(word: str) -> str:
    """Return the SHA-256 of a word, lowercased, as hex.

    Parameters
    ----------
    word : str
        The word.

    Returns
    -------
    str
        The digest.
    """
    return hashlib.sha256(word.lower().encode()).hexdigest()


def _files() -> list[str]:
    """Return every file git would commit, relative to the repository, except this one.

    Returns
    -------
    list of str
        The paths, tracked or new and not ignored, that exist on disk.
    """
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=REPO,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return [f for f in out.splitlines() if f != THIS and (REPO / f).is_file()]


def _mentions(text: str) -> list[tuple[int, str, str]]:
    """Return each forbidden mention in a text.

    Parameters
    ----------
    text : str
        The text.

    Returns
    -------
    list of tuple of (int, str, str)
        Each mention's line (from 1), what it is, and the text matched.
    """
    found = []
    for lineno, line in enumerate(text.splitlines(), 1):
        for m in PATTERN.finditer(line):
            assert m.lastgroup is not None
            found.append((lineno, WHAT[int(m.lastgroup[1:])], m.group()))
    return found


def _names(text: str, name: tuple[str, ...]) -> list[int]:
    """Return the line of each place a name's words appear in a text, in order.

    The words may be split by anything that is not a letter (spaces, a line break, a dot), so
    a name wrapped across lines or written into an address is found too.

    Parameters
    ----------
    text : str
        The text.
    name : tuple of str
        The SHA-256 of each of the name's words, lowercased.

    Returns
    -------
    list of int
        The line (from 1) of each occurrence's first word.
    """
    words = list(_WORD.finditer(text))
    lines = []
    for i in range(len(words) - len(name) + 1):
        if all(_digest(words[i + k].group()) == name[k] for k in range(len(name))):
            lines.append(text.count("\n", 0, words[i].start()) + 1)
    return lines


def _problems(
    rel: str, data: bytes, hidden: Mapping[str, tuple[tuple[str, ...], ...]] = HIDDEN
) -> list[str]:
    """Return what the guard reports in one file.

    Parameters
    ----------
    rel : str
        The file's path, relative to the repository (``ALLOWED`` is keyed by it).
    data : bytes
        The file's content.
    hidden : mapping of str to tuple of tuple of str, optional
        What is held as digests, by what it is reported as (``HIDDEN``'s form).

    Returns
    -------
    list of str
        One ``path:line: what (match)`` per forbidden mention that ``ALLOWED`` does not exempt
        in this file, and one ``path:line: what``, never echoing the text, per occurrence of a
        spelling held as digests; none for a binary file (one holding a NUL byte).
    """
    if b"\0" in data:
        return []  # binary
    text = data.decode("utf-8", errors="replace")
    problems = [
        f"{rel}:{n}: {what} ({hit!r})"
        for n, what, hit in _mentions(text)
        if (rel, hit) not in ALLOWED
    ]
    problems += [
        f"{rel}:{n}: {what}"
        for what, spellings in hidden.items()
        for spelling in spellings
        for n in _names(text, spelling)
    ]
    return problems


def test_no_tracked_file_names_a_place_on_one_machine_or_its_owner() -> None:
    """No file git would commit names a local folder, a personal tool or detail, or the owner."""
    problems = [p for rel in _files() for p in _problems(rel, (REPO / rel).read_bytes())]
    assert not problems, "\n".join(problems)


def test_the_guard_sees_each_kind_of_mention() -> None:
    """Every pattern finds what it is for, labelled so, and leaves alike-looking text alone."""
    cases = [
        ("a = '/home/me/x'", "a home folder"),
        ("cd /home/me", "a home folder"),
        ("b = '/Users/me/x'", "a home folder"),
        ("open '/Users/me'", "a home folder"),
        ("dir C:\\Users\\me\\x", "a home folder"),
        ("c = '~/projects/x'", "a folder under the home folder"),
        ("ls ~/projects", "a folder under the home folder"),
        ("d = '~/.claude/x'", "a folder under the home folder"),
        ("ls ~/.claude", "a folder under the home folder"),
        ("e = '/tmp/claude-1234/x'", "a session's scratch folder"),
        ("f = 'see [[some-note]]'", "a note kept outside the repository"),
        ("g = 'me@Gmail.com'", "a personal email address"),
        ("h = 'https://claude.ai/code/session_0123'", "a link to a Claude Code session"),
    ]
    alike = [
        "i = [[0]]",
        "j = '/tmp/x'",
        "k = '~/.cache/x'",
        "l = '/home/'",
        "m = 'https://claude.ai/code'",
    ]
    found = _mentions("\n".join([line for line, _ in cases] + alike) + "\n")
    assert [(n, what) for n, what, _ in found] == [
        (n, what) for n, (_, what) in enumerate(cases, 1)
    ]  # one mention per case, of its kind; the alike lines give none
    assert "me@Gmail.com" in {hit for _, _, hit in found}  # the whole address, any case


def test_an_allowed_mention_passes_in_its_own_file_alone() -> None:
    """An ``ALLOWED`` (path, match) pair is exempt in its own file and reported in any other."""
    for path, hit in ALLOWED:
        line = f"// Copyright (C) 2009 An Author <{hit}>\n".encode()
        assert _problems(path, line) == []
        # elsewhere it is reported once, as whatever kind of mention it is
        reported = _problems("src/elsewhere.py", line)
        assert len(reported) == 1
        assert reported[0].startswith("src/elsewhere.py:1: ")
        assert reported[0].endswith(f" ({hit!r})")


def test_a_binary_file_is_skipped() -> None:
    """A file holding a NUL byte is not scanned; the same text without it is reported."""
    text = b"x = '/home/me/x'\n"
    assert _problems("a.bin", b"\0" + text) == []
    assert _problems("a.txt", text) == ["a.txt:1: a home folder ('/home/me')"]


def test_the_guard_sees_a_name_by_its_digests() -> None:
    """A name is found by its words' digests, across a line break or a dot, and only in order."""
    name = (_digest("ada"), _digest("lovelace"))
    text = "Ada Lovelace\nwrote to ada.\nLovelace\nLovelace Ada\nada lovelaces\n"
    assert _names(text, name) == [1, 2]


def test_the_guard_reports_each_kind_it_holds_as_digests() -> None:
    """Each kind held as digests is reported as itself, by line, never echoing what it found.

    Stand-ins take the real spellings' places, in their shapes: a folder named ``<name>_project``
    in a path or a string, a tool's folder hyphenated or dotted in the home folder.
    """

    def held(*spellings: str) -> tuple[tuple[str, ...], ...]:
        """Return spellings as ``HIDDEN`` holds them."""
        return tuple(tuple(_digest(word) for word in s.split()) for s in spellings)

    hidden = {
        "the owner's full name": held("ada lovelace"),
        "a folder the project lived in": held("babbage project", "analytical engine project"),
        "a personal tool": held("difference engines", "mill"),
    }
    assert hidden.keys() == HIDDEN.keys()  # a stand-in for every kind the guard holds
    cases = [
        ("by Ada Lovelace", "the owner's full name"),
        ("cd /srv/babbage_project/x", "a folder the project lived in"),
        ("x = 'analytical_engine_project'", "a folder the project lived in"),
        ("see difference-engines/run.py", "a personal tool"),
        ("ls ~/.mill/x", "a personal tool"),
    ]
    alike = [
        "babbage_projects",
        "project babbage",
        "analytical_project",
        "difference engine",
        "mills",
    ]
    text = "\n".join([line for line, _ in cases] + alike) + "\n"
    assert sorted(_problems("a.txt", text.encode(), hidden)) == sorted(
        f"a.txt:{n}: {what}" for n, (_, what) in enumerate(cases, 1)
    )  # one report per case, of its kind; the alike lines give none
    # the real spellings are whole SHA-256 digests, and no kind is also a pattern's
    for spelling in (s for spellings in HIDDEN.values() for s in spellings):
        assert spelling
        assert all(re.fullmatch(r"[0-9a-f]{64}", digest) for digest in spelling)
    assert not HIDDEN.keys() & FORBIDDEN.keys()
