"""Guard: nothing in the repository names a place on one machine, or its owner.

Datasets and runs are named by settings or relative to the repository, so no tracked file
needs a home folder, a session's scratch folder or link, the folders the project has lived in
on one machine, or anything from one person's own tools; and the owner is named by their public
handle alone. This scans the text of every file git would commit (this one excepted, since it
spells out the patterns).

The owner's full name, the folders the project lived in and the personal tools' folders are not
written here. The name is held as the SHA-256 of each of its words, lowercased, and every run of
words in a file is compared with it; each folder as the SHA-256 of its whole name, lowercased,
and every token of a file (a run of letters and digits joined by underscores or hyphens) is
compared with them.
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

# The owner's full name, matched by its words: the SHA-256 of each word, lowercased, in order.
# Anything but a letter parts two words, so the name is found wrapped across lines, written into
# an address, or in prose.
NAME = (
    "ae74f72d212fb9871302a2459aeaf7b20bc2f792e4852be648a7d4e63967d9b1",
    "2be4e33aa3e21f6ff7ad144f8fd58228a1cce3b50340b4805ff0c63480eec488",
)
_WORD = re.compile(r"[^\W\d_]+")

# The folders, by what the guard reports, matched by their whole names: the SHA-256 of each
# name, lowercased, joined as on disk (by underscores or hyphens, a leading dot kept). They are
# found in tokens (`_TOKEN`), as a whole token or inside a longer one (`_folders`), so in a
# path, a string or a file name, but not as the same words in prose.
HIDDEN: dict[str, frozenset[str]] = {
    "a folder the project lived in": frozenset(
        {
            "cc0e56ac7b6bbf4087bfcc44395ba1121733cf36b0081fa5e3d7a660711981e6",
            "3a5cb8d6850913996102877c1788ef70f64ca0cf5d4ec50c7cded95dc959c84d",
            "e1ffc52daaf0d68f8379d0fffd801135ec59e60a6a46cc5a6776db18e7a25f24",
            "205b567d83d2c277ca6c38e5499f9dde7109f89fec9be1d2830225c8cbeedfe5",
        }
    ),
    "a personal tool": frozenset(
        {
            "68bd42e0b3a5ef398398b995a9f1312642049e505407f32b67af6ddb1af05c13",
            "09ca0ac421bfaf2f055e0a3179d13a8708a5e573c4401de6de8f6cdb27c95d30",
        }
    ),
}
_TOKEN = re.compile(r"\.?[a-z0-9]+(?:[_-][a-z0-9]+)*")
# Where a token parts: at an underscore or a hyphen (kept, to join the parts again), and
# between a letter and a digit (with nothing to keep).
_PARTS = re.compile(r"([_-]|(?<=[a-z])(?=[0-9])|(?<=[0-9])(?=[a-z]))")

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


def _sha256(text: str) -> str:
    """Return the SHA-256 of a text as hex, uncached: most runs of a token's parts appear once.

    Parameters
    ----------
    text : str
        The text, already lowercased.

    Returns
    -------
    str
        The digest.
    """
    return hashlib.sha256(text.encode()).hexdigest()


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


def _names(text: str, name: tuple[str, ...] = NAME) -> list[int]:
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


def _folders(text: str, hidden: Mapping[str, frozenset[str]] = HIDDEN) -> list[tuple[int, str]]:
    """Return each place in a text's tokens that is a folder held as a digest.

    Each token is cut into parts (``_PARTS``), and every run of consecutive parts, joined as in
    the token, is compared: so a name is found as a whole token or inside a longer one, after a
    word or before one (``old-<name>``, ``<name>_backup``) or next to digits (``<name>2``). A
    run that starts a token with a leading dot is compared with the dot and without it.

    Parameters
    ----------
    text : str
        The text, lowercased here.
    hidden : mapping of str to frozenset of str, optional
        The SHA-256 of each folder's whole name, by what it is reported as (``HIDDEN``'s form).

    Returns
    -------
    list of tuple of (int, str)
        Each place's line (from 1) and what it is.
    """
    kinds = {digest: what for what, digests in hidden.items() for digest in digests}
    lowered = text.lower()
    found = []
    for m in _TOKEN.finditer(lowered):
        token = m.group()
        pieces = _PARTS.split(token.removeprefix("."))
        parts, joins = pieces[0::2], pieces[1::2]
        for i in range(len(parts)):
            run = parts[i]
            for j in range(i, len(parts)):
                if j > i:
                    run += joins[j - 1] + parts[j]
                dotted = (f".{run}", run) if i == 0 and token.startswith(".") else (run,)
                what = next((kinds[d] for s in dotted if (d := _sha256(s)) in kinds), None)
                if what is not None:
                    found.append((lowered.count("\n", 0, m.start()) + 1, what))
    return found


def _problems(
    rel: str,
    data: bytes,
    name: tuple[str, ...] = NAME,
    hidden: Mapping[str, frozenset[str]] = HIDDEN,
) -> list[str]:
    """Return what the guard reports in one file.

    Parameters
    ----------
    rel : str
        The file's path, relative to the repository (``ALLOWED`` is keyed by it).
    data : bytes
        The file's content.
    name : tuple of str, optional
        The owner's name, as ``NAME`` holds it.
    hidden : mapping of str to frozenset of str, optional
        The folders, as ``HIDDEN`` holds them.

    Returns
    -------
    list of str
        One ``path:line: what (match)`` per forbidden mention that ``ALLOWED`` does not exempt
        in this file, and one ``path:line: what``, never echoing the text, per occurrence of
        the owner's name or of a folder held as a digest; none for a binary file (one holding a
        NUL byte).
    """
    if b"\0" in data:
        return []  # binary
    text = data.decode("utf-8", errors="replace")
    problems = [
        f"{rel}:{n}: {what} ({hit!r})"
        for n, what, hit in _mentions(text)
        if (rel, hit) not in ALLOWED
    ]
    problems += [f"{rel}:{n}: the owner's full name" for n in _names(text, name)]
    problems += [f"{rel}:{n}: {what}" for n, what in _folders(text, hidden)]
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

    Stand-ins take the real names' places, in their shapes: a folder named ``<name>_project``, a
    tool's folder hyphenated or dotted; each in a path, before a dot or at a sentence's end, and
    inside a longer token, after a word, before one or both, or next to digits; never reported as
    the same words in prose. The name is found across a line break.
    """
    name = (_digest("ada"), _digest("lovelace"))
    hidden = {
        "a folder the project lived in": frozenset(
            {_digest("babbage_project"), _digest("analytical_engine_project")}
        ),
        "a personal tool": frozenset({_digest("difference-engines"), _digest(".mill")}),
    }
    assert hidden.keys() == HIDDEN.keys()  # a stand-in for every kind the guard holds
    folder, tool = hidden
    text = (
        "cd /srv/babbage_project/x\n"  # 1: in a path
        "It lives in /data/Analytical_Engine_Project.\n"  # 2: at a sentence's end, any case
        "see difference-engines/run.py\n"  # 3: before a slash
        "ls ~/.mill/x\n"  # 4: dotted, in the home folder
        "log difference-engines.md\n"  # 5: before a dot extension
        "cp -r old-babbage_project /x\n"  # 6: after a word
        "mv babbage_project_backup /x\n"  # 7: before a word
        "ls old_analytical_engine_project_v2\n"  # 8: after and before words, with a digit
        "ls babbage_project2 3difference-engines\n"  # 9: next to digits, twice
        "ls ~/.mill-old/x\n"  # 10: dotted, before a word
        "the babbage project and the analytical engine project\n"  # 11: prose, not reported
        "babbage-project, difference engines, a mill, the mill-pond\n"  # 12: alike, neither
        "by Ada\nLovelace\n"  # 13: the name, across a line break
    )
    reported = [(1, folder), (2, folder), (3, tool), (4, tool), (5, tool), (6, folder)]
    reported += [(7, folder), (8, folder), (9, folder), (9, tool), (10, tool)]
    reported += [(13, "the owner's full name")]
    assert sorted(_problems("a.txt", text.encode(), name, hidden)) == sorted(
        f"a.txt:{n}: {what}" for n, what in reported
    )
    # the real names are whole SHA-256 digests, and no kind is also a pattern's or the name's
    for digest in (*NAME, *(d for digests in HIDDEN.values() for d in digests)):
        assert re.fullmatch(r"[0-9a-f]{64}", digest)
    assert not HIDDEN.keys() & {*FORBIDDEN, "the owner's full name"}
