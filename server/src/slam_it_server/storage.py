"""Where uploads and results are kept: an S3 bucket, or a folder on this machine.

Either hands out links, so that files go straight between a browser or a worker and the store,
never through the server's own process:

- an **upload link** (PUT) signed for one key, an exact size and a life of its own, so the store
  refuses anything else: the size cap is checked when the job is made, against the size the
  browser declares;
- a **download link** (GET) for one key, for :data:`slam_it_server.config.LINK_SECONDS`; S3's,
  given a content type, signed so that the bucket answers with it (a showcase run's calibration
  page, answered as a page).

:class:`S3Storage` signs them with the bucket's keys (boto3); :class:`FolderStorage` with an HMAC
of its own key, the server serving them (``/storage/<key>``), every file as a download. A key is
a relative path of names of letters, digits, ``.``, ``_`` and ``-``, never ``.`` or ``..``: the
server makes every key itself. A failure is raised, never read as "not there" or as done.
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

from slam_it_server.config import LINK_SECONDS

_KEY = re.compile(r"[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+)*")
"""A key the server makes, matched whole: relative, its parts plain names."""


class StorageError(Exception):
    """A key is malformed, or a link's signature does not hold."""


def check_key(key: str) -> str:
    """Return a key, refusing one that is not a plain relative path (the module docstring)."""
    if not _KEY.fullmatch(key) or {".", ".."} & set(key.split("/")):
        raise StorageError(f"key: not a plain relative path: {key!r}")
    return key


def _origin(url: str) -> str:
    """Return a URL's scheme and host, as ``https://host:port``."""
    parts = urllib.parse.urlsplit(url)
    return f"{parts.scheme}://{parts.netloc}"


class Storage(Protocol):
    """What the server needs of a store."""

    @property
    def origin(self) -> str:
        """Return the scheme and host its links point to, for the pages' connections."""
        ...

    def upload_link(self, key: str, size: int, seconds: int) -> str:
        """Return a PUT link for a key, signed for exactly `size` bytes, for `seconds`."""
        ...

    def download_link(self, key: str, content_type: str | None = None) -> str:
        """Return a GET link for a key, answered with a content type where the store can."""
        ...

    def size(self, key: str) -> int | None:
        """Return a key's size in bytes, or None if it is not there."""
        ...

    def move(self, key: str, to: str) -> None:
        """Move a key's file to another key."""
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

    @property
    def origin(self) -> str:
        """Return the scheme and host its links point to: boto3 decides, by its addressing style."""
        params = {"Bucket": self.bucket, "Key": "origin"}
        return _origin(self.client.generate_presigned_url("get_object", Params=params, ExpiresIn=1))

    def upload_link(self, key: str, size: int, seconds: int) -> str:
        """Return a PUT link, its Content-Length signed so the bucket refuses another size."""
        params = {"Bucket": self.bucket, "Key": check_key(key), "ContentLength": size}
        return str(
            self.client.generate_presigned_url("put_object", Params=params, ExpiresIn=seconds)
        )

    def download_link(self, key: str, content_type: str | None = None) -> str:
        """Return a GET link; with a content type, signed so the bucket answers with that type."""
        params = {"Bucket": self.bucket, "Key": check_key(key)}
        if content_type is not None:
            params["ResponseContentType"] = content_type
        return str(
            self.client.generate_presigned_url("get_object", Params=params, ExpiresIn=LINK_SECONDS)
        )

    def size(self, key: str) -> int | None:
        """Return the object's size, or None if it is not there."""
        try:
            head = self.client.head_object(Bucket=self.bucket, Key=check_key(key))
        except self.client.exceptions.ClientError as error:
            if error.response.get("Error", {}).get("Code") in ("404", "NoSuchKey"):
                return None
            raise  # not "absent": the bucket refused or failed
        return int(head["ContentLength"])

    def move(self, key: str, to: str) -> None:
        """Copy the object to the other key, then delete it (S3 has no rename)."""
        source = {"Bucket": self.bucket, "Key": check_key(key)}
        self.client.copy_object(CopySource=source, Bucket=self.bucket, Key=check_key(to))
        self.delete(key)

    def delete(self, key: str) -> None:
        """Delete the object (S3 deletes quietly what is not there)."""
        self.client.delete_object(Bucket=self.bucket, Key=check_key(key))

    def delete_prefix(self, prefix: str) -> None:
        """Delete every object under the prefix, a page of up to 1,000 at a time."""
        pages = self.client.get_paginator("list_objects_v2")
        for page in pages.paginate(Bucket=self.bucket, Prefix=check_key(prefix) + "/"):
            objects = [{"Key": item["Key"]} for item in page.get("Contents", [])]
            if objects:
                done = self.client.delete_objects(Bucket=self.bucket, Delete={"Objects": objects})
                if done.get("Errors"):
                    failed = ", ".join(error["Key"] for error in done["Errors"])
                    raise StorageError(f"delete: the bucket kept {failed}")


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

    @property
    def origin(self) -> str:
        """Return the scheme and host its links point to: this server's public address's."""
        return _origin(self.public_url)

    def _signature(self, method: str, key: str, expires: int, size: int) -> str:
        """Return the HMAC of a link's method, key, expiry and size."""
        message = f"{method}\n{key}\n{expires}\n{size}".encode()
        return hmac.new(self.key, message, hashlib.sha256).hexdigest()

    def _link(self, method: str, key: str, size: int, seconds: int) -> str:
        """Return a signed link to this server's storage route, for `seconds`."""
        expires = int(self.clock()) + seconds
        query = urllib.parse.urlencode(
            {
                "expires": expires,
                "size": size,
                "signature": self._signature(method, key, expires, size),
            }
        )
        return f"{self.public_url}/storage/{urllib.parse.quote(check_key(key))}?{query}"

    def upload_link(self, key: str, size: int, seconds: int) -> str:
        """Return a signed PUT link for exactly `size` bytes, for `seconds`."""
        return self._link("PUT", key, size, seconds)

    def download_link(self, key: str, content_type: str | None = None) -> str:
        """Return a signed GET link.

        A content type is not used: folder storage serves every file as a download, never as a
        page of the server's own site (``/storage/<key>``).
        """
        del content_type
        return self._link("GET", key, -1, LINK_SECONDS)

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

    def move(self, key: str, to: str) -> None:
        """Move the file to the other key."""
        target = self.path(to)
        target.parent.mkdir(parents=True, exist_ok=True)
        self.path(key).replace(target)

    def delete(self, key: str) -> None:
        """Delete the file, if it is there."""
        self.path(key).unlink(missing_ok=True)

    def delete_prefix(self, prefix: str) -> None:
        """Delete the folder of files under the prefix (the server's own key, checked)."""
        folder = self.path(prefix)
        if folder.exists():
            shutil.rmtree(folder)  # a failure raises, for the sweep to try again
