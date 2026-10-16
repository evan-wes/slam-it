"""Train a place-recognition vocabulary (:mod:`slam_it.foundations.vocabulary`) from images.

The images are drawn at random, with a fixed seed, from one or more sources; each is read as the
pipeline reads a frame (gray, :func:`slam_it.eval.images.read_gray`), its ORB features extracted
with the pipeline's settings for its size (:func:`slam_it.orchestration.vo.options_for`, by the
compiled backend, whose descriptors are the Python reference's bit for bit), and every descriptor
kept with its image, for the words' document frequencies. The tree is then built by hierarchical
k-means and written to OUT as a vocabulary's folder (:mod:`slam_it.eval.vocabularies`), its
arrays' SHA-256 in ``sha256.npy``. The same sources, sample, seed and shape give the same tree,
its four integer and byte arrays byte for byte; the weights, numpy's log of each word's document
frequency, may differ in the last bit on another CPU (numpy has its own AVX-512 log).

A source is ``KIND:PATH``:
- ``tar:FILE``: every ``.png`` member of a tar file (Rawseeds' Bovisa: one camera's frames);
- ``zips:FOLDER``: every ``.png`` in the zip files one level below FOLDER (ARKitScenes' raw
  captures, each ``<video_id>/vga_wide.zip``).

Usage:
    python scripts/train_vocabulary.py OUT --source KIND:PATH [--source ...] [--images 10000]
        [--levels 6] [--k 10] [--seed 0] [--name NAME]
for example (paths outside the repository):
    python scripts/train_vocabulary.py ~/vocabularies/bovisa-6 --name bovisa-2008-09-01 \
        --source tar:datasets/bovisa/Bovisa_2008-09-01-FRONTAL.tar --levels 6
"""

from __future__ import annotations

import argparse
import contextlib
import sys
import tarfile
import time
import zipfile
from collections.abc import Callable
from pathlib import Path

import numpy as np

from slam_it.eval import images, vocabularies
from slam_it.foundations import orb, vocabulary
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo

Frame = tuple[str, Callable[[], bytes]]
"""A frame's name, and how to read its bytes."""


def tar_frames(path: Path, stack: contextlib.ExitStack) -> list[Frame]:
    """Every ``.png`` member of a tar file, by name; the file open until `stack` closes."""
    archive = stack.enter_context(tarfile.open(path))  # noqa: SIM115 -- the stack closes it
    members = [m for m in archive.getmembers() if m.isfile() and m.name.endswith(".png")]

    def reader(member: tarfile.TarInfo) -> Callable[[], bytes]:
        def read() -> bytes:
            handle = archive.extractfile(member)
            if handle is None:
                raise InvalidInputError(f"{path}: {member.name} has no data")
            return handle.read()

        return read

    return [(f"{path.name}/{m.name}", reader(m)) for m in members]


def zip_frames(folder: Path, stack: contextlib.ExitStack) -> list[Frame]:
    """Every ``.png`` in the zip files one level below a folder; each open until `stack` closes."""

    def reader(archive: zipfile.ZipFile, name: str) -> Callable[[], bytes]:
        return lambda: archive.read(name)

    out: list[Frame] = []
    for path in sorted(folder.glob("*/*.zip")):
        archive = stack.enter_context(zipfile.ZipFile(path))
        for name in sorted(n for n in archive.namelist() if n.endswith(".png")):
            out.append((f"{path.parent.name}/{name}", reader(archive, name)))
    return out


KINDS = {"tar": tar_frames, "zips": zip_frames}


def main(arguments: list[str]) -> int:
    """Train and write a vocabulary from the command line's sources (the module docstring)."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", type=Path, help="the vocabulary's folder to write")
    parser.add_argument("--source", action="append", required=True, help="KIND:PATH, repeatable")
    parser.add_argument("--images", type=int, default=10_000, help="images drawn (10,000)")
    parser.add_argument("--levels", type=int, default=6, help="the tree's depth (6)")
    parser.add_argument("--k", type=int, default=10, help="a node's most children (10)")
    parser.add_argument("--seed", type=int, default=0, help="the draw's and k-means' seed (0)")
    parser.add_argument("--name", default="", help="the training set's name, for the record")
    args = parser.parse_args(arguments)
    with contextlib.ExitStack() as stack:  # the sources, open while their frames are read
        frames: list[Frame] = []
        for source in args.source:
            kind, _, path = source.partition(":")
            if kind not in KINDS:
                parser.error(f"--source: expected one of {sorted(KINDS)}, got {kind!r}")
            frames += KINDS[kind](Path(path).expanduser(), stack)
        frames.sort(key=lambda f: f[0])
        if len(frames) < args.images:
            parser.error(
                f"--images: the sources hold {len(frames)} frames, fewer than {args.images}"
            )
        drawn = np.sort(
            np.random.default_rng(args.seed).choice(len(frames), args.images, replace=False)
        )
        start = time.perf_counter()
        descriptors, owners = [], []
        for i, index in enumerate(drawn):
            _, read = frames[index]
            gray = images.read_gray(read())
            options = vo.options_for(gray.shape[1], gray.shape[0]).extraction()
            features = orb.extract(gray, options, backend="cpp")
            descriptors.append(features.descriptors)
            owners.append(np.full(len(features.descriptors), i, dtype=np.int64))
            if (i + 1) % 1000 == 0:
                print(f"{i + 1} images, {time.perf_counter() - start:.0f} s", flush=True)
    stacked = np.concatenate(descriptors)
    print(f"{len(stacked)} descriptors from {len(drawn)} images; training", flush=True)
    info = {
        "name": args.name,
        "sources": [
            s.partition(":")[0] + ":" + Path(s.partition(":")[2]).name for s in args.source
        ],
        "drawn_from": len(frames),
    }
    tree = vocabulary.train(
        stacked, np.concatenate(owners), k=args.k, levels=args.levels, seed=args.seed, info=info
    )
    vocabularies.write(args.out, tree)
    print(f"{tree.size} words in {time.perf_counter() - start:.0f} s: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
