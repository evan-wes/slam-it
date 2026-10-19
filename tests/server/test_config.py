"""The server's settings: read from the environment, the template's placeholders among them."""

from __future__ import annotations

from pathlib import Path

import pytest

from slam_it_server.config import SHOWCASE_LENGTH, SettingsError, from_environment

TEMPLATE = Path(__file__).resolve().parents[2] / "server" / "settings.template.env"


def _template() -> dict[str, str]:
    """Return the template's variables."""
    lines = TEMPLATE.read_text().splitlines()
    pairs = [line.split("=", 1) for line in lines if line and not line.startswith("#")]
    return {name: value for name, value in pairs}


def test_the_template_lists_every_setting_and_reads() -> None:
    """Every variable the module reads is in the template, which reads as valid settings."""
    env = _template()
    settings = from_environment(env)
    assert settings.storage == "s3"
    assert set(settings.s3) == {"endpoint", "bucket", "key_id", "secret", "region"}
    assert settings.staging
    source = Path(__file__).resolve().parents[2] / "server/src/slam_it_server/config.py"
    named = {word for word in source.read_text().split() if word.startswith("``SLAM_IT_")}
    assert {w.strip("`:,") for w in named} <= set(env)


def test_folder_storage_needs_no_bucket() -> None:
    """With ``folder:<path>``, the bucket's variables may be left out, and uploads kept."""
    env = {k: v for k, v in _template().items() if not k.startswith("SLAM_IT_S3_")}
    env["SLAM_IT_STORAGE"] = "folder:/somewhere"
    assert from_environment(env).s3 == {}
    assert not from_environment(env).keep_uploads
    assert from_environment({**env, "SLAM_IT_KEEP_UPLOADS": "1"}).keep_uploads


@pytest.mark.parametrize(
    ("name", "value", "message"),
    [
        ("SLAM_IT_WORKER_TOKEN", "", "SLAM_IT_WORKER_TOKEN is not set"),
        ("SLAM_IT_INVITE_CODES", "a-long-enough-code,short", "each code needs 12"),
        ("SLAM_IT_SIGNING_KEY", "too-short", "needs 32 characters"),
        ("SLAM_IT_STORAGE", "disk", "expected s3 or folder"),
        ("SLAM_IT_S3_BUCKET", " ", "SLAM_IT_S3_BUCKET is not set"),
        ("SLAM_IT_KEEP_UPLOADS", "1", "only with folder storage"),
    ],
)
def test_a_missing_or_weak_setting_is_refused(name: str, value: str, message: str) -> None:
    """A variable unset, a code or secret too short, an unknown storage: refused by name.

    So are uploads kept on S3, which only the demo's folder storage may keep.
    """
    env = {**_template(), name: value}
    with pytest.raises(SettingsError, match=message):
        from_environment(env)


def test_the_showcase_is_a_list_of_names() -> None:
    """Unset or empty: none. Its names are read stripped, empty entries skipped, in order."""
    env = _template()
    unset = {name: value for name, value in env.items() if name != "SLAM_IT_SHOWCASE"}
    assert from_environment(unset).showcase == ()
    assert from_environment({**env, "SLAM_IT_SHOWCASE": ""}).showcase == ()
    listed = {**env, "SLAM_IT_SHOWCASE": " tum-fr1-desk, room ,,euroc-mh01"}
    assert from_environment(listed).showcase == ("tum-fr1-desk", "room", "euroc-mh01")
    longest = "a" * SHOWCASE_LENGTH
    assert from_environment({**env, "SLAM_IT_SHOWCASE": longest}).showcase == (longest,)


@pytest.mark.parametrize(
    ("value", "message"),
    [
        ("Room", "not a name"),
        ("room_1", "not a name"),
        ("a/b", "not a name"),
        ("..", "not a name"),
        ("a.b", "not a name"),
        ("-room", "not a name"),
        ("room-", "not a name"),
        ("a--b", "not a name"),
        ("a b", "not a name"),
        ("%2e", "not a name"),
        ("a" * (SHOWCASE_LENGTH + 1), "not a name"),
        ("room,tum,room", "listed twice"),
    ],
)
def test_a_showcase_name_is_plain_short_and_listed_once(value: str, message: str) -> None:
    """A name with capitals, underscores, slashes, dots, spaces or escapes: refused, by name.

    So are hyphens at an end or doubled, a name too long, and a name listed twice.
    """
    with pytest.raises(SettingsError, match=f"SLAM_IT_SHOWCASE: .*{message}"):
        from_environment({**_template(), "SLAM_IT_SHOWCASE": value})
