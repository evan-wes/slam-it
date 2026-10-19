"""A dashcam's recordings as SLAM It! datasets: joined, masked, clocked, calibrated, imported.

A dashcam records each drive as short segments per camera, burns its GPS fix and clock into
every frame, and sees the car's own hood, shelf and wires in every frame. This package turns a
card's copy into sequences a run can take, and a GPS track to score them against:

- :mod:`.d016`: the Terunsoul D016's formats, the one dashcam profile built in.
- :mod:`.drives`: a card's segments, grouped into drives, front and rear paired.
- :mod:`.mask`: the static mask, measured from a drive (edges that never move).
- :mod:`.overlay`: the burned-in overlay read with Tesseract: the GPS fixes, and each camera's
  clock against its video.
- :mod:`.stream`: the GPS records in each segment's data stream (speed and course).
- :mod:`.calibrate`: each camera's lens, and the front-rear rig, from each camera's own motion
  (COLMAP through pycolmap).
- :mod:`.ingest`: one camera's share of a drive as one TUM-format sequence.
- :mod:`.config`: the config that holds where a card is and what was measured of it, kept with
  the recordings, not in the repo (``docs/dashcam.template.toml``).

``python -m slam_it.eval.dashcam`` runs the steps (:mod:`.__main__`). Tesseract, pytesseract and
pycolmap are the optional ``dashcam`` extra, imported only where they are used (an
import-linter contract keeps them there); PyAV decodes through :mod:`slam_it.eval.video` alone.
See ``docs/notes/dashcam.md``.
"""
