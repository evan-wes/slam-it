"""Storage's keys and links: folder storage's HMAC, and S3's presigned links, signed offline."""

from __future__ import annotations

import urllib.parse
from pathlib import Path

import boto3
import pytest
from botocore.config import Config

from slam_it_all_server.config import LINK_SECONDS
from slam_it_all_server.storage import FolderStorage, S3Storage, StorageError, check_key
from tests.server.conftest import Clock


@pytest.mark.parametrize(
    "key", ["", "/abs", "a/../b", "..", "a//b", "a/", "a b", "a\\b", "a/./b/%2e%2e", "é"]
)
def test_a_key_that_is_not_a_plain_relative_path_is_refused(key: str) -> None:
    """Absolute, climbing, empty parts, or any other character: refused."""
    with pytest.raises(StorageError):
        check_key(key)


def test_a_plain_key_is_kept() -> None:
    """Letters, digits, dots, underscores, dashes and slashes between names."""
    assert (
        check_key("results/0a1b/cpp/feature_records.npy") == "results/0a1b/cpp/feature_records.npy"
    )


def _query(link: str) -> tuple[str, dict[str, str]]:
    """Return a link's path and its query."""
    parts = urllib.parse.urlsplit(link)
    return parts.path, dict(urllib.parse.parse_qsl(parts.query))


def test_a_folder_link_holds_only_as_signed(tmp_path: Path) -> None:
    """Its method, key, size and expiry are signed: change any, or wait, and it is refused."""
    clock = Clock()
    store = FolderStorage(tmp_path, "http://here", b"k" * 32, clock)
    path, query = _query(store.upload_link("uploads/a", 10))
    assert path == "/storage/uploads/a"
    expires, signature = int(query["expires"]), query["signature"]
    assert expires == clock.now + LINK_SECONDS
    store.verify("PUT", "uploads/a", expires, 10, signature)
    for method, key, size in [
        ("GET", "uploads/a", 10),
        ("PUT", "uploads/b", 10),
        ("PUT", "uploads/a", 11),
    ]:
        with pytest.raises(StorageError, match="signature"):
            store.verify(method, key, expires, size, signature)
    with pytest.raises(StorageError, match="signature"):
        store.verify("PUT", "uploads/a", expires + 1, 10, signature)
    other = FolderStorage(tmp_path, "http://here", b"j" * 32, clock)
    with pytest.raises(StorageError, match="signature"):
        other.verify("PUT", "uploads/a", expires, 10, signature)
    clock.now = expires + 1
    with pytest.raises(StorageError, match="expired"):
        store.verify("PUT", "uploads/a", expires, 10, signature)


def test_folder_storage_sizes_and_deletes(tmp_path: Path) -> None:
    """A file's size, None for none; a key and a prefix deleted, quietly if not there."""
    store = FolderStorage(tmp_path / "files", "http://here", b"k" * 32)
    store.path("results/j/a.npy").parent.mkdir(parents=True)
    store.path("results/j/a.npy").write_bytes(b"12345")
    store.path("uploads/j").parent.mkdir(parents=True)
    store.path("uploads/j").write_bytes(b"1")
    assert (store.size("results/j/a.npy"), store.size("results/j/b.npy")) == (5, None)
    store.delete("uploads/j")
    store.delete("uploads/j")
    store.delete_prefix("results/j")
    store.delete_prefix("results/j")
    assert (store.size("uploads/j"), store.size("results/j/a.npy")) == (None, None)
    assert (tmp_path / "files").is_dir()


def _s3() -> S3Storage:
    """Return S3 storage with made-up keys: boto3 signs without a network."""
    client = boto3.client(
        "s3",
        endpoint_url="https://bucket.example.invalid",
        aws_access_key_id="AKIDEXAMPLE",
        aws_secret_access_key="a-made-up-secret",
        region_name="auto",
        config=Config(signature_version="s3v4"),
    )
    return S3Storage(client, "uploads-bucket")


def test_an_s3_upload_link_signs_its_size() -> None:
    """The PUT link signs Content-Length, so the bucket refuses another size; it lasts an hour."""
    path, query = _query(_s3().upload_link("uploads/job", 123_456))
    assert path.endswith("/uploads/job")
    assert query["X-Amz-Algorithm"] == "AWS4-HMAC-SHA256"
    assert "content-length" in query["X-Amz-SignedHeaders"].split(";")
    assert int(query["X-Amz-Expires"]) == LINK_SECONDS


def test_an_s3_download_link_is_signed_for_its_key() -> None:
    """The GET link names its key and is signed, for an hour."""
    path, query = _query(_s3().download_link("results/job/bundle.json"))
    assert path.endswith("/results/job/bundle.json")
    assert int(query["X-Amz-Expires"]) == LINK_SECONDS
    assert len(query["X-Amz-Signature"]) == 64
    with pytest.raises(StorageError):
        _s3().download_link("results/../uploads/job")
