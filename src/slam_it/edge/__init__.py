"""Edge rung: sparse visual odometry for real-time use on edge devices (ORB-SLAM-lite).

Milestone 2's pipeline, in monocular, stereo and RGB-D modes, on rung 0's ORB front end:
tracking, keyframes and local bundle adjustment, as ORB-SLAM2 does them without its second
thread (design doc, "The edge rung is ORB-SLAM-lite"; the plan is
``docs/design/2026-10-05-edge-vo.md``). Milestone 2B makes it ORB-SLAM's own
(``docs/design/2026-10-14-online-slam.md``): from B1, a frame tracking fails is relocalised
against the whole map through a bag of words; from B2, the local map, local mapping and
relocalisation's candidates take their keyframes from the covisibility graph; from B3, each
keyframe mapped is searched for a loop through the same bag of words, and a loop closed pulls
the map onto it: a Sim(3), the essential graph and a global BA, on rung 0's factor graphs.

- :mod:`slam_it.edge.options`: every setting, ORB-SLAM's defaults.
- :mod:`slam_it.edge.frames`: a frame's features, bearings, depth and right columns, per mode.
- :mod:`slam_it.edge.map`: keyframes, their keypoints and the map points, a struct of arrays.
- :mod:`slam_it.edge.covisibility`: the covisibility graph, counted from the observations when
  asked, and the spanning tree.
- :mod:`slam_it.edge.database`: the keyframe database, each keyframe's words and the inverted
  index from each word to its keyframes (ORB-SLAM's ``KeyFrameDatabase``).
- :mod:`slam_it.edge.residuals`: each level's variance and the chi-square outlier test.
- :mod:`slam_it.edge.adjust`: bundle adjustment over keyframes and the points they observe.
- :mod:`slam_it.edge.initialization`: starting the map, from depth or from two views.
- :mod:`slam_it.edge.tracking`: placing each frame against the map, and the fallback.
- :mod:`slam_it.edge.relocalization`: placing a frame the fallback fails in the whole map,
  through the database's candidates (ORB-SLAM's ``Tracking::Relocalization``).
- :mod:`slam_it.edge.mapping`: growing and cleaning the map around each new keyframe.
- :mod:`slam_it.edge.loops`: detecting the place a keyframe revisits, through the database's
  candidates, and correcting the map onto it (ORB-SLAM3's ``LoopClosing``).
- :mod:`slam_it.edge.threads`: the threads' handshakes, when the steps run in threads of their
  own (milestone 2B's B3b).
- :mod:`slam_it.edge.atlas`: the run's records across its maps: the maps' numbers, every
  point's and keyframe's life, and each frame's place for the final trajectory.
- :mod:`slam_it.edge.pipeline`: the per-frame loop over them: tracking, local mapping and
  loop closing as steps over state of their own, and the owner of the map being tracked.

**Compute budget:** real time under ``laptop-cpu``, then ``jetson-class`` and ``pi-class``.
In order, the default, local mapping and loop closing run inline after tracking, and mapping
falls behind on the largest maps since milestone 2B's B2 (its load 1.7 to 2.6 on the test set's
five largest); both are judged by their loads as threads of their own would carry them, and
mapping also by the backlog its thread would build (:func:`slam_it.eval.budgets.judge`).
Threaded (milestone 2B's B3b, :mod:`slam_it.edge.threads`), tracking, local mapping, loop closing
and a global BA each run in a thread of their own: up to four threads, against ``laptop-cpu``'s
4 cores, which a budget takes as physical cores (:func:`slam_it.eval.budgets.cpus`). Mapping
keeps up as ORB-SLAM's does, refusing keyframes and cutting its local BA short, and the judge
reads it so: the keyframes asked for and refused, the queue, the local BAs aborted and
tracking's wait. CPU only; implemented in Python, C++ and Rust in lock-step. Must never import
PyTorch, a GPU stack or a depth model (enforced by import-linter). Builds only on
:mod:`slam_it.foundations`.
"""
