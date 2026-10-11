"""Where uploads and results are kept: an S3 bucket, or a folder on this machine.

Either hands out links, so that files go straight between a browser or a worker and the store,
never through the server's own process:

- an **upload link** (PUT) signed for one key and an exact size, so the store refuses anything
  else: the size cap is checked when the job is made, against the size the browser declares;
- a **download link** (GET) for one key.

Links last :data:`slam_it_all_server.config.LINK_SECONDS`. :class:`S3Storage` signs them with
the bucket's keys (boto3); :class:`FolderStorage` with an HMAC of its own key, the server
serving them (``/storage/<key>``). A key is a relative path of letters, digits, ``.``, ``_``,
``-`` and ``/``, without ``..``: the server makes every key itself.
"""

from __future__ import annotations

import hashlib
import hmac
import re
import shutil
import time
import urllib.parse
from pathlib import Path
from typing import Any, Protocol

from slam_it_all_server.config import LINK_SECONDS

_KEY = re.compile(r"^[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+)*$")
"""A key the server makes: relative, its parts plain names."""


class StorageError(Exception):
    """A key is malformed, or a link's signature does not hold."""


def check_key(key: str) -> str:
    """Return a key, refusing one that is not a plain relative path (the module docstring)."""
    if not _KEY.match(key) or ".." in key.split("/"):
        raise StorageError(f"key: not a plain relative path: {key!r}")
    return key


class Storage(Protocol):
    """What the server needs of a store."""

    def upload_link(self, key: str, size: int) -> str:
        """Return a PUT link for a key, signed for exactly `size` bytes."""
        ...

    def download_link(self, key: str) -> str:
        """Return a GET link for a key."""
        ...

    def size(self, key: str) -> int | None:
        """Return a key's size in bytes, or None if it is not there."""
        ...

    def delete(self, key: str) -> None:
        """Delete a key, if it is there."""
        ...

    def delete_prefix(self, prefix: str) -> None:
        """Delete every key under a prefix (a folder of keys)."""
        ...


class S3Storage:
    """A private S3 bucket (Railway's, or any S3-compatible one), through boto3.

    Parameters
    ----------
    client : Any
        A boto3 S3 client.
    bucket : str
        The bucket.
    """

    def __init__(self, client: Any, bucket: str) -> None:  # noqa: ANN401 -- boto3 has no types here
        """Hold the client and the bucket."""
        self.client, self.bucket = client, bucket

    def upload_link(self, key: str, size: int) -> str:
        """Return a PUT link, its Content-Length signed so the bucket refuses another size."""
        params = {"Bucket": self.bucket, "Key": check_key(key), "ContentLength": size}
        return str(
            self.client.generate_presigned_url("put_object", Params=params, ExpiresIn=LINK_SECONDS)
        )

    def download_link(self, key: str) -> str:
        """Return a GET link."""
        params = {"Bucket": self.bucket, "Key": check_key(key)}
        return str(
            self.client.generate_presigned_url("get_object", Params=params, ExpiresIn=LINK_SECONDS)
        )

    def size(self, key: str) -> int | None:
        """Return the object's size, or None if it is not there."""
        try:
            head = self.client.head_object(Bucket=self.bucket, Key=check_key(key))
        except self.client.exceptions.ClientError:
            return None
        return int(head["ContentLength"])

    def delete(self, key: str) -> None:
        """Delete the object (S3 deletes quietly what is not there)."""
        self.client.delete_object(Bucket=self.bucket, Key=check_key(key))

    def delete_prefix(self, prefix: str) -> None:
        """Delete every object under the prefix, a page of up to 1,000 at a time."""
        pages = self.client.get_paginator("list_objects_v2")
        for page in pages.paginate(Bucket=self.bucket, Prefix=check_key(prefix) + "/"):
            objects = [{"Key": item["Key"]} for item in page.get("Contents", [])]
            if objects:
                self.client.delete_objects(Bucket=self.bucket, Delete={"Objects": objects})


class FolderStorage:
    """A folder on this machine, its links signed with an HMAC and served by the server.

    Parameters
    ----------
    root : pathlib.Path
        The folder.
    public_url : str
        The server's address, which the links point at.
    key : bytes
        The signing key.
    clock : callable, optional
        Seconds since the epoch (default: :func:`time.time`).
    """

    def __init__(self, root: Path, public_url: str, key: bytes, clock: Any = time.time) -> None:  # noqa: ANN401
        """Hold the folder, the address and the signing key."""
        self.root, self.public_url, self.key, self.clock = root, public_url.rstrip("/"), key, clock
        root.mkdir(parents=True, exist_ok=True)

    def _signature(self, method: str, key: str, expires: int, size: int) -> str:
        """Return the HMAC of a link's method, key, expiry and size."""
        message = f"{method}\n{key}\n{expires}\n{size}".encode()
        return hmac.new(self.key, message, hashlib.sha256).hexdigest()

    def _link(self, method: str, key: str, size: int) -> str:
        """Return a signed link to this server's storage route."""
        expires = int(self.clock()) + LINK_SECONDS
        query = urllib.parse.urlencode(
            {
                "expires": expires,
                "size": size,
                "signature": self._signature(method, key, expires, size),
            }
        )
        return f"{self.public_url}/storage/{urllib.parse.quote(check_key(key))}?{query}"

    def upload_link(self, key: str, size: int) -> str:
        """Return a signed PUT link for exactly `size` bytes."""
        return self._link("PUT", key, size)

    def download_link(self, key: str) -> str:
        """Return a signed GET link."""
        return self._link("GET", key, -1)

    def verify(self, method: str, key: str, expires: int, size: int, signature: str) -> None:
        """Check a link the server is asked to serve.

        Raises
        ------
        StorageError
            If the key is malformed, the link has expired, or the signature does not hold.
        """
        check_key(key)
        if expires < self.clock():
            raise StorageError("link: expired")
        if not hmac.compare_digest(signature, self._signature(method, key, expires, size)):
            raise StorageError("link: the signature does not hold")

    def path(self, key: str) -> Path:
        """Return a key's file."""
        return self.root / check_key(key)

    def size(self, key: str) -> int | None:
        """Return the file's size, or None if it is not there."""
        file = self.path(key)
        return file.stat().st_size if file.is_file() else None

    def delete(self, key: str) -> None:
        """Delete the file, if it is there."""
        self.path(key).unlink(missing_ok=True)

    def delete_prefix(self, prefix: str) -> None:
        """Delete the folder of files under the prefix (the server's own key, checked)."""
        shutil.rmtree(self.path(prefix), ignore_errors=True)
