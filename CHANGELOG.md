# Changelog

All notable changes to this project are documented here. Format is a
loose variant of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
with 4-digit `MAJOR.MINOR.PATCH.MICRO` version numbers.

## [0.3.17.0] - 2026-10-19

Milestone 2B's B4: maps merged, in the three languages (`docs/design/2026-10-14-online-slam.md`,
section 14). The atlas keeps every map the track leaves whole, each with its keyframe database;
relocalisation searches every kept map with the one tracked; loop closing searches each keyframe for
a merge with a kept map beside its loop, by ORB-SLAM3's stages, and merges the map tracked into the
one it matched, with the paper's correction (a welding BA over both windows, the essential graph
over the whole merged map) and global BA after. A run can take several sequences in turn, each a
session that starts a new map. The run format is version 9: runs written in version 8 no longer
read; the bundle's is version 5, so the showcase's runs are baked and uploaded again before a
release serves this viewer. Threaded, loop closing finds a merge and tracking makes it between
two frames, pausing for the whole merge (option B, decided by evan-wes on 2026-10-18); a merge that
does not block tracking is the later design. Section 15's measurements (multi-session EuRoC, the
kidnap, 7-Scenes, the loss window, and the test set and the loop sequences) are not run for this
release (decided by evan-wes on 2026-10-18). B4's review panel's findings were decided by
evan-wes on 2026-10-19 and fixed in the PR, or recorded where they are tracked
(`docs/reviews/2026-10-19-feature-2b-b4-merging-maps.md`).

### Added

- **The atlas keeps maps whole** (`slam_it.edge.atlas`, with C++ and Rust twins): a map the track
  leaves is kept with its keyframes, keypoints, points, observations, links and keyframe database
  (`Kept`), or reset if it is too small; each map keeps its ids' run ids, which a merge joins.
  Every point, keyframe and final pose records the map it ends in, and each frame the map it was
  tracked in. A merge carries the active map's records into the matched map's world, its frames'
  and culled keyframes' poses relative to their keyframes into its units (the panel's L1). A
  merged map that is reset drops the frames of the maps merged into it with its own, as a reset
  map's frames are not in the trajectory (A12). A kept map that is not there is `InvalidInput`
  naming its number, in all three languages, at the atlas (H2) and at the merge's lookup of the
  matched map (H15).
- **Relocalisation in every map** (`relocalization.relocalize_across`, with
  `database.merge_candidates` and `database.Indexed`): each map's database finds its own
  candidates, its words counted within the map (B1's panel, L7), all tried best score first. A
  frame found in a kept map goes on in it, the map left kept or reset as a lost map is, recorded
  as the frame's `left`.
- **Merge detection** (`loops.detect_merge`, `loops.merge_first`): ORB-SLAM3's merge branch
  beside loop detection, by the same stages: up to 3 candidates from the other maps, none below
  `loops.min_keyframes`, a merge confirmed as a loop is. A merge found on a keyframe goes before
  a loop closed on it, the loop's outcome `merged` (code 6). Culling spares the last keyframe of a
  merge being confirmed.
- **The merge** (`loops.merge`, `loops.merge_transform`): the active map carried into the matched
  map's world by the merge's Sim(3) and taken in after it, its ids moved on by the matched map's
  next ones; its spanning tree re-rooted and hung under the matched keyframe; the welding windows
  (`loops.weld_neighbours` 25, `loops.weld_widen` 12, `loops.weld_rounds` 5); the matched
  window's points fused into the active window's keyframes; the welding BA with a Huber loss
  (`graphs.bundle_adjust`'s `huber`, in the three languages); the essential graph over the whole
  merged map, both windows held; global BA after, under `loops.global_ba`; and a merge edge
  (`map.add_merge_edge`), apart from the loop edges. A loop being confirmed is carried into the
  merged map, in its units: `diag(1/s, 1/s, 1/s, 1) S G⁻¹` (the panel's L2).
- **Sessions** (`Pipeline.start_session`, `--sequence` repeated, `vo.session_offsets`): a run over
  several sequences of one camera in turn, each starting a new map, the map before reset with
  fewer than `loss.session_keyframes` (12) keyframes and kept otherwise, as ORB-SLAM3's
  `ChangeDataset`. A session whose times do not come after the last one's is moved on, and each
  session's first frame records `session`.
- **Threaded merging, tracking paused for the merge** (option B; `_Threads.merge_waiting` and
  `TrackingState.merged`, with C++ and Rust twins): loop closing's thread leaves a merge found
  waiting (`LoopState.waiting`), and tracking makes it before its next frame, or as the run
  finishes: mapping's queue mapped first, loop closing passing those keyframes by; then, with
  mapping stopped, a global BA still running superseded, the merge made as inline without global
  BA, a global BA started on the merged map only if one was superseded, and the merged view
  published with a new correction count. Tracking's last frame takes the merged map's ids, its
  motions G's scale. A merge waiting when the track leaves its map is dropped.
- **A loss window of 0** (`loss.relocalize_seconds` may be 0): a map above `loss.reset_keyframes`
  is lost on its first failed frame, and the next frame starts a new map.
- **The run format, version 9:** `tracked_maps.npy` and `keyframe_merges.npy`; a run's maps no
  longer need to rise; the diagnostics' `loop` gains `merging` and `merge`, and a frame `left`
  and, on a session's first, `session`.
- **The bundle, version 5:** each run's merges (`keyframe_merges.npy`), which the viewer draws
  with the covisibility graph in a colour of their own (`--graph-merge`). The map at a frame is
  the map it ends in: the records' `map` is written through each merge's destination, read from
  the run's merge records, and a run whose maps disagree (one map's frames ending in two maps,
  by its merges and by its trajectory) or a record naming a map the run neither ends with nor
  merged is refused (A12, H21); the header lists the maps a run ends with, each map's `frames`
  as stretches, and the viewer draws
  a map through the runs of rows its poses come in, and finds a loop's keyframes in a map taken
  up again by the k-th of the map's run ids, not the first's offset (the panel's A1). A run of
  several sequences in turn is refused by name (H13).
- **A 7-Scenes importer** (`slam_it.eval.sevenscenes`, `python -m slam_it.eval.sevenscenes SCENE
  OUT`): a scene's sequences to TUM format, monocular, for section 15.
- **The test-set measure takes sessions** (`scripts/measure_testset.py`): a sequence's `folders`
  run in turn and scored against their ground truths in turn; each result's `sessions`, `maps`
  and `merges`, each merge judged as a loop is (`slam_it.eval.loops.judge_pairs`). A config
  gives `folders` alone, a list of one for a single sequence: `folder` is refused (A10), the
  refusal naming the keys missing and unknown (H22); the measure and `compare_testsets.py`
  share the sessions' offsets (`offsets_of`, H12).

### Changed

- **On B3b's threads:** relocalisation into a kept map and a session's start wait for local
  mapping and loop closing to finish the keyframes given, and supersede a global BA running, as
  a lost map does; the native programs feed each session's first frame at once. Taking up a kept
  map and a merge count as a correction, so the view after carries a new count; threaded, a merge
  waits for tracking, which makes it between two frames (the design's section 14, item 4).
- **`slam_it.eval.loops.judge_loops` is `judge_pairs` over a run's loops:** the loop and merge
  rooms' tests and the measure judge loops and merges by it.
- **A map's ids keep their run ids in a table, not a base** (`Atlas.run_ids`,
  `Atlas.point_run_ids`), so a tracked frame's reference keyframe and its keypoints' points are
  reported by the run's ids through a merge.
- **From B4's review panel:** Rust's loop closing and relocalisation read the kept maps with the
  atlas's lock let go (shared, `Arc`), so threaded tracking never waits on a search (A2), a kept
  map a search still holds never taken out but refused, naming it, where it was copied out (H20,
  A13), and `loops::detect_merge` and `merge_candidates` take the maps by reference in C++ and Rust, as
  relocalisation does; C++ and Rust keep each kept map as one `Kept`, its run ids with it (H2);
  `start_session` refuses a finished run in all three languages (H4); `eval.runs.tracking` gives
  a segment per stretch tracked in one map, and the worker's summary counts the maps a run ends
  with apart (`maps`, which the status page's "maps" row shows, where it showed the segments),
  and the frames relocalisation found in a map kept from earlier (`taken_up`, which the status
  page says apart from those found again in their map) (A11); `Crossing` is `slam_it.backends`'
  alone (H10); each weld setting's doc stands alone (H7); and why a map closes is named
  (`_Closing`), in place of a keyframe count with `+ 1` (H8).

### Tests

- **The merge room** (`tests/orchestration/test_merge_room.py`): the loop room blinded for 3.2 s
  from 90 degrees round, so its first map is lost and kept and a second starts. At
  `MERGE_OPTIONS`, every language ends in one map, merged at no wrong place, its final ATE over
  the merged map and its points' distance to the walls within bounds set from the first runs.
- **Two maps, merged** (`tests/edge/test_merges.py`, on `TwoMaps`): merge candidates,
  relocalisation in every map, merge detection and its confirmation, `merge_first` and the merge,
  each compiled backend held to Python's reference; the welding BA's Huber loss
  (`tests/foundations/test_graphs.py`); merge edges and culling's spared keyframes.
- **The merge room threaded** (`tests/orchestration/test_merge_room_threads.py`): each native
  program, C++ in both schemes, merges it in every mode and holds the merge room's bar, its first
  frame after the merge placed by the motion model; Python's run, in process, each frame waiting
  for its threads, keeps tracking's last frame naming the same keyframe and points through the
  merge, and leaves no thread. ThreadSanitizer runs it threaded in both C++ schemes
  (`tests/orchestration/test_merge_room_tsan.py`, in `scripts/check.sh`'s ThreadSanitizer step).
  Known flaky: its TUM monocular case in C++'s own scheme failed in one of three full layer-1
  runs, its first frame after the merge placed by relocalisation, and passed in three runs
  alone; it is in the ladder's open items, beside the loop room's flaky threaded case.
- **Sessions** (`tests/orchestration/test_sessions.py`): the room run twice in turn through every
  backend, the offsets, the shared camera and the refusals; `start_session` in each language's
  pipeline tests; a loss window of 0 in each language's loss rules.
- **The rooms' golden records are regenerated, one per language:** 25 of each record's 27 cases
  are unchanged; window/tum/monocular and window/euroc/monocular change in their fates and stage
  summaries alone (merge detection from frame 22, relocalisation searching the kept map on
  frames 23 to 25), their states, maps and poses unchanged.
- The golden bundles are rebaked at version 5 with all three programs; the run fixture is at
  format 9.
- `slam_it.eval.loops` judges merges as it judges loops (`tests/eval/test_loops.py`).
- The positional test paths are collected in path order, so no directory is collected twice
  (`tests/conftest.py`): pytest lost a sub-directory's conftest fixtures otherwise
  (pytest-dev/pytest#14635, in 9.1.1; `tests/test_collection_order.py`).
- **From B4's review panel:**
  - **The revisit room** (`tests/orchestration/test_revisit_room.py`, the panel's A3): the merge
    room with loop closing off, blinded again for 1 s before the camera comes back over the
    first map's start; every language, inline, and each native scheme, threaded, takes the
    first map up again by relocalisation (`left` recorded, the maps tracked in going 0, 1, 0),
    and ThreadSanitizer runs it threaded in both C++ schemes.
  - **The atlas through a monocular merge** (`tests/edge/test_atlas.py`,
    `cpp/tests/edge/test_atlas.cpp`, `rust/slam-it-edge/tests/atlas.rs`): frames placed through
    the merge, through a culled keyframe too, and the removed points and keyframes, against
    `rigid_of(T G⁻¹)`; a merged map reset, taking both maps' frames (A12); in Rust, a kept map
    a snapshot still holds refused, and moved out once let go (H20); a loop being confirmed
    carried through the merge and confirmed on the next keyframe (`tests/edge/test_merges.py`),
    and in Rust the carry itself, a loop of scale 1.25 keeping it, against its known answer
    (`carry_through`, `pipeline.rs`'s own tests).
  - `held()` and `TrackingState.merged` with a scaled `G` in each language (the native ones
    H19), a finished run's session refused in each language, and the Huber loss's known answer
    on one stereo observation in every backend (`initial_error` 10.0702).
  - **Two golden bundles** (`merge_bundle/`, `revisit_bundle/`), on which the viewer's tests
    choose the map shown (`viewer/tests/maps.test.js`). They leave out their frames' JPEG
    chunks, which no test reads, keeping the images' spans the page's loader reads; the layout
    test compares them without the chunks.

## [0.3.16.2] - 2026-10-19

The two tests that failed the Test workflow on `main` no longer depend on the machine running
them, and the workflow has more time.

### Fixed

- **`tests/eval/test_budgets.py`**: two tests of `budgets.cpus` faked the CPUs offered and the
  performance cores but read the host's hyperthread layout, so they failed on a runner whose
  CPUs 2 and 3 are one core. They now pass an absent topology, each CPU a core of its own on
  any machine.

### Changed

- **`.github/workflows/test.yml`**: the job's time limit is 180 min, not 120; its last run took
  118 min.

## [0.3.16.1] - 2026-10-19

The showcase's home page shows only its runs: the demo shows the run viewer alone until public
uploads are ready.

### Changed

- **`viewer/showcase.html`** no longer has its "Your own video" section linking the upload page;
  the upload page still answers at `/upload`, with its invite code as before, unlinked from the
  showcase. `tests/server/test_showcase.py` asserts the home page does not link it; the note and
  the run-viewer design's section 10 say so.

## [0.3.16.0] - 2026-10-19

The upload server shows runs we baked ourselves: a showcase, named by `SLAM_IT_SHOWCASE`, each
run's bundle kept in storage under `showcase/<name>/`. With one, the home page lists its runs,
each opened in the run viewer by its name, and the upload page moves to `/upload`; without one,
the server behaves as before (`docs/notes/your-own-video.md`, "The showcase";
`docs/design/2026-10-09-run-viewer.md`, section 10).

### Added

- **The showcase setting** (`SLAM_IT_SHOWCASE`, `Settings.showcase`): the runs' names,
  comma-separated, each of lowercase letters and digits joined by single hyphens, at most 40
  characters (`config.SHOWCASE_NAME`, `SHOWCASE_LENGTH`); a name malformed or listed twice stops
  the server at startup. `server/settings.template.env` lists it, empty, and
  `scripts/serve_demo.py` sets it empty, so the demo never takes one from its shell.
- **The showcase's page at `/`** (`viewer/showcase.html`, `app.showcase_page`): each run by its
  name, linked to `/s/<name>/`, and a link to the upload page. In the upload page's style and with
  its head, with no script and no inline style, under the pages' content policy as it is. The
  server reads it once at startup and puts the links in place of its `<!-- runs -->` marker
  (`app.SHOWCASE_MARKER`); a page missing, or its marker missing or repeated, stops it. It is not
  served by its own name; the server's image copies it, and layer 1's prettier checks it.
- **`/s/<name>/`** (and `/s/<name>`): the run viewer on the run's bundle,
  `/index.html?bundle=%2Fs%2F<name>%2Fbundle%2F`, the link the status page makes
  (`app.VIEWER_PAGE`, held equal to `viewerLink` in `viewer/src/jobs.js`).
- **`/s/<name>/bundle/<path>`:** redirected to storage's link for `showcase/<name>/<path>`
  (`app.showcase_prefix`), for a bundle's names and kinds of file alone (`RESULT_PATH`, as a
  job's result), by a link made for each request.
- **`/s/<name>/calibration.html`:** the run's calibration page, kept beside its bundle
  (`app.CALIBRATION_PAGE`), redirected by a link that asks the bucket to answer it as a page
  (`app.CALIBRATION_TYPE`, `text/html; charset=utf-8`). It is self-contained, its images
  embedded, and has no script; it renders at storage's address, without the site's headers. No
  other file beside the bundle is served.
- **The upload page at `/upload`,** with a showcase or without.

### Changed

- **Storage's download links take a content type** (`Storage.download_link(key,
  content_type)`), for the calibration page: S3's asks the bucket to answer with it
  (`ResponseContentType`); folder storage, which serves every file as a download, leaves it out.
- A done job's bundle files and a showcase run's are redirected by one function
  (`app._stored`); the pages' `Cache-Control: no-cache` is one constant (`app._PAGE_HEADERS`).
- `make_s3`, the tests' S3 storage signed offline, moves from `tests/server/test_storage.py` to
  `tests/server/conftest.py`, so the showcase's tests use it too.
- `bundles.VERSION`'s docstring says that a new bundle version means baking the showcase's runs
  again and uploading them again.

### Tests

- The server's tests go from 131 to 188.
- `tests/server/test_showcase.py` (41): the home page with a showcase and without, and the upload
  page at `/upload`; the showcase's page with the upload page's head, no script or inline style,
  and one marker; a page that cannot be filled refused before the job list is made (3); the
  server's image copying it; a run opened in the viewer, under the content policy; its bundle's
  files redirected, a new link made after the first expired; the calibration page redirected to
  be answered as HTML (S3) and served as a download (folder storage); 29 addresses answered 404
  (a name not listed, a file of another name, kind or folder, images kept beside the bundle, the
  showcase's page by name, and paths that climb or are absolute, escaped or not); the job list
  never used; staging's Basic auth over the showcase.
- `tests/server/test_config.py` gains 13 (the names read, and 12 refused),
  `tests/server/test_storage.py` 1 (S3's content type), `tests/server/test_shared_names.py` 1 (the
  viewer's link) and `tests/server/test_app.py` 1 (`/upload`); `tests/test_serve_demo.py`'s
  settings test checks the demo's showcase is empty.

## [0.3.15.0] - 2026-10-19

The run viewer's controls, and the loops a run closed made visible: sliders for the 3D panel's
keyframe size and map point opacity, value axes on the Solver panel's plots, a Solver panel that
holds its height during playback, and each loop closed marked on the timeline, shown beside the
earlier pass at the same place, and flashed over the map when the playhead arrives at it.

### Added

- **A keyframe size slider** in the 3D panel's head: logarithmic, from 2^-4.3 (about 1/20) to 2
  times each map's own frustum depth, ten steps a doubling (`settings.js`), its value shown
  ("×0.25") and remembered in the browser; it scales the camera's frustum too. Each map's own
  depth is now capped at twice the median distance between its consecutive keyframes, still at
  least `MIN_DEPTH` (`geometry.mapDepths`, `SPACING_SHARE`), so dense keyframes along a long run
  no longer overlap; a run whose trajectory's share (1/40 of its extent) is the smaller keeps
  its look.
- **A map point opacity slider** beside it: 5% to 100% in steps of 5, 100% by default,
  remembered; below 100% the points blend and write no depth.
- **Value axes on the Solver panel's plots** (`axis.js`): a gutter with ticks from 0, a step of 1,
  2, 2.5 or 5 times a power of ten apart, each labelled, faint grid lines across, and the unit
  ("points"; "ms" on the latency plot, whose values are seconds). The deadline's dashed line is
  labelled with its frame rate, "deadline 40 ms (25 fps)" (`format.deadline`).
- **Loops on the timeline** (`loops.js`, `loop_view.js`): each loop the chosen run closed is
  ticked in a row of its own under the backends' rows, at the frame whose record holds its
  correction, checked against `keyframe_loops.npy`; its tooltip names its keyframes as the
  records count them, in their map, and the time between their making. "◀ loop" and "loop ▶" in
  the transport bar jump to the loop before and after the frame shown.
- **"Seen before":** from 3 s before each loop's frame to 5 s after it, an inset in the Frame
  panel's corner shows the earlier pass at the same place. Each frame takes the earlier frame
  whose final pose's centre is nearest its own, among its map's frames within 10 s of the
  matched keyframe's making, and never steps back, so the inset plays forward, labelled "seen
  N s earlier · keyframe K". A click enlarges it; a toggle in the panel head, remembered, turns
  it off. Its frames load through the main image's loader (`FrameImages`), with a cache of
  their own.
- **A loop's flash:** arriving at a loop (playing onto or across its frame, a seek onto it, the
  loop buttons) draws its cycle (its map's trajectory from the matched keyframe's making to the
  loop), the edge between its keyframes and their frustums over the map, thick in the loops'
  colour (three.js's `Line2` and `LineSegments2`), for 1.5 s, then fades them over 1 s on a
  clock of its own; meanwhile the inset plays the earlier pass from the matched keyframe on, at
  the frames' own speed. A seek elsewhere or another backend ends it; a new loop replaces it.

### Changed

- **The Solver panel's tables are built once a run,** with every stage the run summarises at any
  frame and all its fields, and a frame shown changes only their text ("–" where a stage did not
  run, its heading muted), so the panel holds its height during playback
  (`solver.summaryLayout`, `summaryAt(records, k, layout)`). Each value is one line, cut with an
  ellipsis and whole on hover; "this run did not run the frame" takes the panel head's place of
  "placed by".
- **The import map pins three's five line modules** (`examples/jsm/lines/`) by their sha384, as
  it pins the others.
- **The playhead tells its callback whether playback moved it or a seek did,** which the loop's
  flash tells apart.

### Tests

- The viewer's tests go from 61 to 84.
- `viewer/tests/loops.test.js` (12): the C++ runs of the rooms' golden records that close loops
  (`tests/fixtures/edge/room_runs_cpp.json`), held to what their own columns say (each frame
  whose summary holds a correction, its keyframes the `keyframe_loops` row), with their tooltips
  where every frame is tracked; a run with no loop column; a made-up run of two maps whose
  record counts keyframes in its map, and loops the run does not hold; the loop beside a frame;
  the inset's earlier frames on a made-up street driven twice and on the loops of the rooms
  whose every frame is tracked (never stepping back, before the cutoff, nearer the loop's place
  than the window's start), the window chosen where two overlap, and no earlier frames for no
  loops or a loop whose map has no tracked poses; a loop's cycle; the flash's opacity over time
  and the frames the inset plays meanwhile; and what each move of the playhead does to the
  flash.
- `viewer/tests/axis.test.js` (4): ticks and their labels, tops that are no positive number, the
  finest step, and the deadline's text; `viewer/tests/settings.test.js` (2): both sliders'
  ranges and the values read from storage.
- `viewer/tests/solver.test.js` gains 2 (the run's rows, and every record's shape),
  `viewer/tests/images_geometry.test.js` 2 (the depth cap) and `viewer/tests/playback.test.js` 1
  (playback or a seek).

## [0.3.14.0] - 2026-10-18

Milestone 2B's B3b: tracking, local mapping and loop closing in threads of their own, in the three
languages (`docs/design/2026-10-14-online-slam.md`, section 12). Tracking runs in the caller's
thread and reads, with no lock, a view of the map that its one writer at a time publishes; local
mapping, loop closing and each global BA run in threads of their own, scheduled as ORB-SLAM
schedules them. ORB-SLAM's own locking runs in C++ as the baseline to compare with. In order, in
one thread, stays the default, and such a run repeats exactly: the rooms' golden records are
unchanged. The run format is version 8: runs written in version 7 no longer read. Section 13's
measurements (the latency bar, the accuracy threaded, and our design against ORB-SLAM's scheme)
are not run for this release (decided by evan-wes on 2026-10-18).

### Added

- **The seams** (`slam_it.edge.atlas`, with C++ and Rust twins): the run's records leave the
  pipeline for the atlas; tracking, local mapping and loop closing become steps over state of
  their own; the map being tracked and its keyframe database are reached through one owner
  (`MapOwner`). No decision changes.
- **The threads' handshakes** (`slam_it.edge.threads`, C++'s `slam_it/edge/threads.hpp`, Rust's
  `slam_it_edge::threads`): one writer of the map at a time, stopped, held and released as
  ORB-SLAM's `RequestStop` and `Release`; the queue between two threads; the global BAs'
  generation; and a worker thread whose failure is kept for the thread that gave it work.
- **The pipeline in threads of its own** (Python's `Pipeline(threaded=True)`, the `Threading`
  that C++'s `Pipeline::create` and Rust's `Pipeline::new` take, the programs' `--threaded`):
  tracking in the caller's thread, on the view it took for its frame; local mapping, loop
  closing and each global BA in threads of their own; a loop searched with mapping held off and
  corrected with it stopped; global BA solving a copy of the map, stopped when a newer loop
  supersedes it, its result otherwise carried onto the map through the spanning tree
  (`loops.propagate_global_ba`); a lost map closed once the threads have finished its
  keyframes. Threaded, the native programs feed the frames at their recorded rate.
- **A view of what tracking reads:** the writers publish the keyframes tracking's next frame may
  read (`covisibility.view_keyframes`), copied into a map of their own (`map.sub_map`), when a
  map starts, after a keyframe's points are made, when its mapping ends and after each
  correction; Python's view is its immutable map. A test mode, the programs' `--views`, tracks
  each frame in order on the view a writer would publish then.
- **ORB-SLAM's scheduling:** a keyframe is asked for only while mapping accepts one; with depth,
  a busy mapping's local BA is interrupted and keyframes queue up to `mapping.max_queued` (3), a
  monocular one is refused; fusion, local BA and culling run only while no keyframe waits; the
  keyframe rule's busy branch (`keyframes.busy_ref_ratio`, 0.25).
- **Solves that stop between iterations,** as g2o's `setForceStopFlag` stops ORB-SLAM's: rung 0's
  least-squares solve and the factor graphs' Levenberg-Marquardt end with the termination
  "stopped" (nls code 7, graphs code 3); local BA aborts between iterations, and global BA takes
  a stop.
- **ORB-SLAM's locking as C++'s baseline** (`Threading::kLocks`, `slam_it_vo --threaded
  --locks`): the map's lock taken where ORB-SLAM takes a keyframe's or a point's, so stricter
  than ORB-SLAM, with the same scheduling as our design.
- **The run format, version 8:** `run.json`'s `threaded` and `locks`, and each frame's
  handshakes (`slam_it.eval.runs.HANDSHAKES`): `wait_seconds`, `asked`, `refused`, `queue`, its
  keyframe's `schedule`, the global BAs that ended on it (`global_ba`, `global_ba_seconds`) and
  the views published while it was tracked (`views`).
- **The budget judge for threads:** a threaded run's mapping judged by the keyframes asked for
  and refused, the largest queue and the local BAs aborted, with tracking's wait for the map; a
  run in order also by the backlog a mapping thread would build, by Lindley's recursion (PR 9's
  panel, L8); a budget's cores pinned as physical cores (L12). `scripts/measure_testset.py`
  takes `--threaded`, `--locks` and `--budget` and records section 13's measures;
  `scripts/measure_sharded.py` takes `--cores`, and its workers run on one logical CPU of each
  physical core, by default as many workers as those cores make, where they took every CPU.

### Changed

- **`mapping.need_keyframe` takes whether mapping is idle, the frames since the last keyframe
  and the map's keyframes,** where it assumed mapping always idle; `mapping.process` takes what
  its thread is asked between its steps, and says how each step was scheduled.
- **Layer 1 builds the C++ tests once more with ThreadSanitizer** (`build/tsan`) and runs them
  with `halt_on_error`; the native threaded tests run both schemes. It also runs the loop room
  threaded, RGB-D, through the instrumented program in both schemes, so that a loop's stop and
  correction and a global BA's thread run under the sanitizer
  (`tests/orchestration/test_loop_room_tsan.py`, marked `tsan` and left out of the main pytest
  step; the review panel's A3).
- **From B3b's review panel** (`docs/reviews/2026-10-18-feature-2b-b3b-threads.md`):
  - a view carries the corrections applied to its map, stamped by its writer, and tracking
    reads it once a frame, so the last frame follows each correction (A1);
  - a keyframe tracked before a correction, queued while it ran or asked for just after, is
    placed by its reference keyframe as the map now has it, `T_frame_reference
    T_reference_world`, through culled ancestors (`Atlas.pose_by`); the design's section 12
    records why, against ORB-SLAM3's `EmptyQueue` (A2);
  - Python's threaded owner drops a lost map's waiting counts (H1); its mapping is idle only
    with no keyframe waiting or in hand (H4); a failed run ends its threads, through
    `Pipeline.close()` and `vo`'s in-process run, and so does a pipeline dropped without
    `finish()` (H5);
  - a panic in a Rust thread is kept as an `InvalidInput` error naming the thread, the item
    done all the same, so draining never hangs, and a global BA's panic is no longer dropped
    (H6, A4);
  - `scripts/measure_sharded.py` checks `--cores` first and names `--jobs x --cores` (H7);
  - `mapping.need_keyframe`'s new arguments, `budgets.backlog` and
    `loops.propagate_global_ba` refuse a bad argument by its name (H9);
  - a view's fallback to the whole map is asserted against in debug builds (H13).

### Tests

- The handshakes' order, held by events rather than timing, in each language
  (`tests/edge/test_threads.py`, `cpp/tests/edge/test_threads.cpp`,
  `rust/slam-it-edge/tests/threads.rs`).
- The threaded pipeline in each language: every frame tracked, each keyframe mapped once, the
  records whole, a finished run refusing frames, a stop refusing every keyframe asked, and a
  lost map closing as in order (`tests/edge/test_pipeline_threads.py` and the native pipeline
  tests).
- The loop room's bar runs threaded in C++ and Rust too, fed at its rate
  (`tests/orchestration/test_loop_room.py`).
- `--views` runs equal the runs in order on all 27 golden-record cases, in C++ and Rust;
  `view_keyframes` and `sub_map` on every backend.
- The run fixture is a threaded run with locks and handshakes, which the C++ and Rust writers
  write value for value; the golden bundles are rebaked.
- After a correction made under a stop, the next frame is placed by the motion model in the
  corrected world, and a keyframe queued across it is placed by its reference, in each language
  and both C++ schemes; a failed or dropped threaded Python run leaves no thread; a panicking
  Rust step is an error, not a hang; and Python's handshake tests fail when a thread's
  assertion does (H2).

## [0.3.13.0] - 2026-10-17

A dashcam's recordings as SLAM It! datasets (`slam_it.eval.dashcam`, `docs/notes/dashcam.md`),
built for the Terunsoul D016, and a GPS track to score a run against (`slam_it.eval.gnss`).
The owner's recordings, masks, lenses and results stay outside the repo, in a config whose
template is `docs/dashcam.template.toml`.

### Added

- **`slam_it.eval.dashcam`**, run as `python -m slam_it.eval.dashcam <step> CONFIG`:
  - `drives`: a card's segments grouped into drives, front and rear paired;
  - `mask`: a camera's static mask measured from a drive (persistent edges), with drawn
    parts and a preview;
  - `overlay`: the burned-in overlay read with Tesseract into a fix a second and each
    camera's lag (its overlay clock against its segments' names);
  - `calibrate`: each camera's fisheye lens self-calibrated by COLMAP's sequential SfM, then
    the front-rear rig: the clocks' offset from the two cameras' turns, the rotation from
    their turns' axes and directions of travel, and the translation and rear scale by a
    linear MAP with a baseline prior, the front map's scale from the GPS speed;
  - `import`: one camera's share of a drive as one TUM-format sequence on its overlay clock.
  The D016's formats are a built-in profile (`dashcam.d016`), its data stream's GPS records
  read for their time, speed and course (`dashcam.stream`). Every step skips a damaged
  segment (`eval.video.DamagedSegmentError`: one that cannot be read through) with a
  warning and goes on, `overlay` and `import` listing those skipped. `mask`, `overlay` and
  `calibrate` refuse to replace what they wrote before, and each step checks everything it
  would write, and the earlier steps' outputs it reads, before it writes anything, so a
  refusal leaves nothing half done.
- **`slam_it.eval.gnss`**: a gated constant-velocity Kalman filter with an RTS smoother over
  GNSS fixes, and a run's maps scored against the track in their plane of motion, in metres.
- **The `dashcam` extra** (PyAV, pytesseract, pycolmap) and Tesseract in the conda layer;
  import-linter contracts keep pytesseract in `dashcam.overlay` and pycolmap in
  `dashcam.calibrate` (and the BA reference), and the pipelines out of both.
  `THIRD_PARTY_NOTICES.md` lists Tesseract 5.5.3 and pytesseract 0.3.13, both Apache 2.0.
- **`eval.video` reads recorded segments**: sampled in time, their data packets, and joined
  into one sequence (`import_segments`), sharing the video import's frame writing, which can
  now take back a damaged segment's frames.

### Tests

- Synthetic only: MPEG-TS segments written with PyAV (a moving scene under a static hood,
  a data stream of records, a burned-in overlay), the drives, the mask, the record decoder,
  the drive import and its timing, the config and the command line; the overlay's reading
  with Tesseract (skipped without it) and the lag from reads; the filter's gating, restart
  and smoothing and the planar scoring; each calibration stage on a synthetic rig drive; a
  damaged segment skipped by each step, one failing partway taken back, all damaged refused,
  and a segment with concealed corrupt frames kept; and `mask`, `overlay` and `calibrate` each
  refusing before they write: a preview left from before, the rear's overlay already read, the
  rear's overlay not yet read.
- The guard against local paths and personal details (`tests/test_no_local_paths.py`) holds
  the folders the project lived in and the personal tools' folders as SHA-256 digests, as it
  holds the owner's name, so it no longer spells them out. A folder is held by its whole name
  and found in a token, whole or inside a longer one, in a path, a string or a file name, but
  not as the same words in prose; the name is still found by its words, across a line break
  too. Its self-test has a stand-in for each kind it holds so, and its scratch-folder case is a
  generic one.

## [0.3.12.0] - 2026-10-17

Milestone 2B's third part, B3: loop closure (`docs/design/2026-10-14-online-slam.md`, section
10). Each keyframe, after local mapping, is searched for a loop in its map, as ORB-SLAM3
searches; a loop closed is corrected, its essential graph optimised and the whole map
bundle-adjusted, on GTSAM 4.3 in the three languages. The trajectory a run scores is ORB-SLAM's
final one. On the loop sequences, 16 of 22 runs meet a bar anchored to ORB-SLAM's published
accuracy, in C++ and Rust alike, and no loop closes at a wrong place; on the test set, 18 loops
close in 14 runs, one at a wrong place. Each language keeps its own golden record and is held to
a bar of results on the same rooms, not to the others' decisions. The run format is version 7
and the bundle's 4: runs written by 0.3.11.0 no longer read.

### Added

- **GTSAM 4.3 as the back end** (`slam_it.foundations.graphs`, with C++ and Rust twins, Rust
  through a `cxx` bridge whose factors call back into Rust): a pose graph, the Sim(3)
  refinement and bundle adjustment. `scripts/setup_slam_it.sh` builds GTSAM 4.3.0 from its tag
  with `EIGEN_NO_CPUID`, as the cores compile, so a run repeats on any core, and its Python
  wrapper over the same library; the worker's image builds the C++ library the same way, for
  its C++ program, and its Python, which imports GTSAM but never solves with it,
  takes PyPI's 4.3.0 wheel, pinned by its hash (`scripts/locks/worker-gtsam.lock`).
- **The loop's Sim(3) by RANSAC** in the three languages, each sample solved by Umeyama's
  method, its matches tested as ORB-SLAM's `Sim3Solver` tests them.
- **Loop closing** (`slam_it.edge.loops`, with C++ and Rust twins), its settings in the `loops`
  group: ORB-SLAM3's detection (`DetectNBestCandidates`, matching by word, the loop's Sim(3),
  projection, refinement, and confirmation by the new keyframe's neighbours and the keyframes
  after it); ORB-SLAM's correction (`CorrectLoop`: the new keyframe and its links moved by the
  Sim(3), the matched points replacing theirs, the loop's points fused); the essential graph;
  and global BA after each loop (`loops.global_ba`). Each stage is held to Python's reference
  in C++ and Rust.
- **The map's loop edges;** culling spares a keyframe with one, or one that detection holds.
- **The final trajectory:** each frame placed by its reference keyframe's final pose, through
  its culled ancestors; the poses as tracked are kept beside it.
- **The run format, version 7, and the bundle's, version 4:** `trajectory.txt` holds the final
  trajectory and `trajectory_tracked.txt` the poses as tracked; `keyframe_loops.npy`; a frame's
  `loop_seconds`; each keyframe's detection and each loop's correction in the diagnostics. The
  viewer draws the loop edges, and its Solver panel describes loop closing.
- **The budget judge** counts the keyframes searched for a loop, spreads their loop-closing
  times, and gives loop closing's load.
- **The loop room:** a camera circling 420° in 280 frames on a circle of 0.6 m, looking
  outward and up, so a loop closes in every mode; and the bar on the rooms for the three
  languages in layer 1 (`tests/orchestration/test_loop_room.py`): the frames tracked, one map,
  a loop closed and none at a wrong place, the final trajectory's error and the points' distance
  to the walls.
- **The test-set measure** reads EuRoC; scores the final trajectory (`ate`), the poses as
  tracked (`ate_tracked`) and the final keyframes (`ate_keyframes`); judges each loop against
  the ground truth, at a wrong place by the distance between its keyframes and the angle
  between their viewing directions, and with a poor pose; and records its runs' details, so
  that measures compare across a format break (B2's panel, A9).
- **B3's results**, in `docs/notes/test-set.md` and the design's section 11: on the loop
  sequences, 16 of 22 runs within 1.5 times ORB-SLAM's published ATE with at least 95% of their
  frames tracked; on the test set, 209,148 frames tracked in C++ against B2's 208,906; global
  BA lowering the final ATE on 4 loop sequences and raising it on 4.

### Changed

- **A golden record per language** (`tests/fixtures/edge/room_runs_{python,cpp,rust}.json`, 27
  cases each), each its own language's regression check, in place of one record that every
  language had to reproduce; the tests that compared whole runs across languages are gone, and
  each native's reading of its settings is held in its own language.
- **Every bundle-adjustment factor gives a point behind its camera GTSAM's constant error,**
  twice the focal length a row, with no gradient; without it, global BA collapsed a monocular
  map.
- **Python's global BA takes GTSAM's own factors through a pinhole camera,** which model it
  exactly; GTSAM's Python wrapper copies the problem's values on every call into a custom factor.
- **The keyframe database is kept whenever relocalisation or loop closing is on**
  (`Options.keeps_keyframe_database`, with C++ and Rust twins), as ORB-SLAM always keeps its
  `KeyFrameDatabase`: before, `loops.enabled` without relocalisation closed no loop, silently.
  A run with `loops.enabled` alone now needs a vocabulary and records it (`vo.run` gives the
  committed one), so `--option relocalization.enabled=false` no longer turns loop closing off;
  add `loops.enabled=false` for neither. `vo.check` and both native programs refuse with
  "relocalization.enabled or loops.enabled needs one" and "given, but relocalization.enabled and
  loops.enabled are both off".
- **The loops' judge is `slam_it.eval.loops`,** moved out of `scripts/measure_testset.py`: the
  test-set measure and the loop room judge a loop alike.
- **The graphs refuse what they took:** `refine_similarity` a `pixels2` row neither finite nor
  wholly NaN, in the three languages; Python's graphs raise `NotConvergedError` where GTSAM
  fails, as C++ and Rust return `NotConverged`; loop closing's `search_by_similarity` checks its
  window and distance, and `essential_graph` its inputs.
- **GTSAM stays behind `foundations.graphs`:** an import-linter contract and `check_layers.py`
  hold it there, its C++ headers included by `graphs.cpp` and the Rust bridge's source alone;
  `_cpp` exports none of its symbols.
- **Offline CI compiles GTSAM's Python wrapper in a scope of its own, under 16 GB,** the rest
  under 8 GB; the GitHub workflow's timeout is 120 minutes. `compare_testsets.py summarize`
  reads each sequence's ground truth as its dataset.
- **`THIRD_PARTY_NOTICES.md` covers B3's code:** GTSAM 4.3.0, built by the setup script, and
  with it CCOLAMD, METIS and Spectra, among what you install; `cxx` among the Rust crates; and
  in the ORB-SLAM credit, loop closing and the essential graph, and `slam_it.foundations.sim3`
  and `slam_it.foundations.graphs` following `Sim3Solver` and `Optimizer`'s solves, on GTSAM.
- **`loops.py`'s list of departures from ORB-SLAM** caps the links' links "as ORB-SLAM's
  design intends; its source adds them all", where it said "as ORB-SLAM's code means to",
  which read like a port the code does not bear out (the licence audit's D1).

### Tests

- Loop closing's stages held to Python's reference in C++ and Rust on the loop room's map: the
  searches, the refinement, the fusion, detection, the correction and global BA, the refusals
  and the outcomes' codes.
- The final trajectory follows the keyframes in each mode, each tracked frame once, in order;
  the run format's final and as-tracked trajectories are written and read in the three
  languages, each refusing a loop edge between two maps.
- The measure: EuRoC read, the three trajectories scored, a loop judged at a wrong place and
  with a poor pose, and a keyframe rolled about its view judged in the right place.
- A parity fixture for rung 0's factor graphs (`tests/fixtures/foundations/graphs.json`): pose
  graphs, both rounds of the Sim(3) refinement and two BAs, from the Python reference; every
  backend held to it, its summaries included, each field under its own policy.
- Every program's diagnostics hold Python's key paths and value types, on the lost, found and
  loop rooms.
- The refusals: Python's loop-closing functions each refuse a bad argument by its name, and
  each core a `pixels2` row neither finite nor wholly NaN. A loop closes with relocalisation off
  (Python, the loop room); with loop closing alone, no failed frame relocalises, in each core.
- `tests/test_build_copies.py` holds the worker image's GTSAM build to the setup script's, and
  rung 0's crate's lints to the workspace's.
- The guard against local paths and personal details (`tests/test_no_local_paths.py`) also
  reports a link to a Claude Code session, and its self-test has a case for it.

## [0.3.11.2] - 2026-10-18

SLAM It! is relicensed under the PolyForm Noncommercial License 1.0.0, to prepare for going
public. Its history was rewritten and its GitHub repository recreated on 2026-10-18, so every
commit carries the new licence. Code and data taken from other projects keep their own
licences, listed in `THIRD_PARTY_NOTICES.md` with their texts in `LICENSES/`. Nothing computes
differently: besides comments, the only change to the cores moves the Rust 3x3 SVD, unchanged,
into a file of its own.

### Added

- **`docs/history/`:** the first repository's 22 pull requests, which GitHub no longer shows
  (`pull-requests.md`: each one's title, branch, merge date, size, merge commit and
  description), and `commit-map.txt`, which maps its 531 commit IDs to their IDs in this
  history, so the IDs quoted in review reports, notes and commit messages resolve.
- **`THIRD_PARTY_NOTICES.md`**, from a licence audit of the code and data: the code under other
  licences (stb_image, MIT; OpenCV's ORB pattern and homography decomposition, and PoseLib's
  P3P solvers, BSD 3-Clause; the Rust 3x3 SVD translated from Eigen, MPL 2.0), the methods
  implemented from others' work (ORB-SLAM, DBoW2, OpenCV's EPnP, Ceres, PCG; no ORB-SLAM code
  is included), the data in the repository with its sources' licences, and what the setup
  installs. **`LICENSES/`** holds the 6 licence texts, and the `slam-it` wheel carries both
  (`license-files`).
- **A Licence section in the README:** the PolyForm licence, the notices and `LICENSES/`.

### Changed

- **The licence is the PolyForm Noncommercial License 1.0.0,** in place of the Apache License
  2.0: the history rewrite put it in every commit's `LICENSE`, the manifests name it
  (`PolyForm-Noncommercial-1.0.0`), and so does the ladder design doc.
- **The history was rewritten and the GitHub repository recreated** on 2026-10-18. Commit IDs
  changed (`docs/history/commit-map.txt`), and pull requests in the new repository start again
  at 1.
- **`THIRD_PARTY_NOTICES` is `THIRD_PARTY_NOTICES.md`,** and the design doc, the front-end
  note, `orb.py` and stb's README name it so. Earlier entries here and the saved reviews keep
  the old name.
- **Eigen's 3x3 Jacobi SVD in Rust has a file of its own,**
  `rust/slam-it-foundations/src/linalg/jacobi_svd3.rs`: it is translated from Eigen 5.0.1, so
  it is under the Mozilla Public License 2.0, which applies file by file, and carries the
  licence's notice and Eigen's copyright lines. It moved unchanged, and its callers name the
  module (`linalg::jacobi_svd3::svd3`, `singular_values3`), as `ba::solve`'s do; both still give
  Eigen's factors bit for bit.
- **The native P3P files name PoseLib** (Viktor Larsson, BSD 3-Clause), whose solvers they
  follow, in a header line, as `p3p.py`'s docstring does; **the homography files name OpenCV's
  `HomographyDecompInria`** (BSD 3-Clause) the same way, in their headers and in
  `homography.py`'s decomposition section.
- **The ARKitScenes data is under Apple's ARKitScenes licence** in the phone-robust VO and
  online-SLAM design docs, which gave CC BY-NC-SA 4.0, its licence until 2024-09-10.
- **The online-SLAM design doc's B3, B3b and B4 sections** say their decisions follow
  ORB-SLAM's papers and a reading of its source, where they said "from" its source files.
- **The review panel cites paths relative to the repository root,** never absolute or
  home-relative, since its reports are committed: the reviewer agents, the skill, its finding
  format and its report template say so.

### Tests

- **A guard against local paths and personal details** (`tests/test_no_local_paths.py`): no
  file git would commit names a home folder (Linux, macOS or Windows), a session's scratch
  folder, the folders the project lived in, personal tools or notes, a gmail address, or the
  owner's full name (held as digests, never written). Four more tests check the guard itself:
  each pattern and the kind it reports, the name's digests, the allowed mentions (each exempt in
  its own file alone), and the skipping of binary files.

## [0.3.11.1] - 2026-10-17

A feature mask: a run can leave out the parts of its frames that never move with the scene,
such as a dashcam's burned-in overlay and the car's own body, in the three languages alike.
Without a mask, every run is as before.

### Added

- **A feature mask** (Python, C++, Rust): a TUM-format folder's optional `mask.png`, 0 where
  no feature is to be tracked. A run drops each feature whose keypoint's nearest pixel is 0,
  after extraction (`edge.frames.unmasked`; the frame builders' `mask`; a stereo pair's left
  image only), so ORB extraction and its parity are unchanged. The readers refuse a mask that
  is not the camera's size or not a readable 8-bit PNG.
- **The video import takes a mask** (`import_video(mask=...)`, `--mask PNG`) at the video's
  stored size, and writes it carried through the frames' scaling, fisheye undistortion and
  rotation; `source.json` records it.

### Tests

- Masked features dropped, held to Python's reference in C++ and Rust; the readers'
  refusals; the import's mask through a fisheye lens and a rotation landing where the frames'
  pixels do; a short run with a mask in each language; golden-record cases with no mask
  reproducing exactly in all three.

## [0.3.11.0] - 2026-10-16

Milestone 2B's second part, B2: the covisibility graph (`docs/design/2026-10-14-online-slam.md`,
section 8). The window of the last 10 keyframes is gone: tracking's local map, the local
fallback, local BA, keyframe culling, triangulation and fusion take their keyframes from the
graph, as ORB-SLAM's do, in the three languages alike. On the test set at seed 0, 208,906 of
246,529 frames are tracked against B1's 200,897, in 226 maps against 268, and none of the
monocular runs the ladder's L15 found lost deep into a map is lost. But local mapping, inline
and never busy, makes more keyframes and adjusts every link of each: judged as a thread of its
own, it falls behind on the largest maps, which B3's design takes up. The settings are grouped
by stage, each named `group.field`, and the run format is version 6: runs written by 0.3.10.0
no longer read.

### Added

- **The covisibility graph** (`slam_it.edge.covisibility`, with C++ and Rust twins): two
  keyframes' weight, the points both observe, counted from the map's observations when asked;
  a keyframe's links, those sharing at least `covisibility.min_points` (15), or its best if
  none does, most shared first; tracking's local map, K1 and K2 as the papers describe it, to
  `covisibility.local_keyframes` (80), and the frame's reference keyframe; the strong links of
  `covisibility.strong_points` (100). Held to a brute-force fixture in the three languages.
- **The spanning tree:** each keyframe's parent (`Map.kf_parent`), the keyframe it shares the
  most points with when inserted; a culled keyframe's children given new parents as ORB-SLAM's
  `SetBadFlag` gives them, by `covisibility.cull`, the one way a keyframe leaves the map:
  `map.erase_keyframe` refuses one that is still a parent, and `cull` the map's first. Kept for
  B3's essential graph.
- **What the stages take from the graph,** public in the three languages and held to the
  fixture's brute force: `mapping.fuse_targets`, fusion's targets, and
  `mapping.local_ba_keyframes`, the newest keyframe and its links.
- **The run format, version 6:** `keyframe_parents.npy` and `keyframe_links.npy`, each
  keyframe's last parent and each map's strong links, written by every language.
- **The viewer draws the graph** on a "graph" toggle: each keyframe's line to its parent and the
  strong links, in the map shown, each alive while both its keyframes are; bundle format 3.
- **The diagnostics:** the local map's keyframes and points and the reference keyframe; local
  BA's free and fixed keyframes; the Solver panel describes them.
- **The test set's B2 results**, in `docs/notes/test-set.md`:
  - 8,009 more frames tracked than B1, in 42 fewer maps, and losses from 231 to 177; the losses
    that began with a local map left too few inliers fall from 152 to 91;
  - none of L15's 20 monocular runs on TUM and EuRoC is lost, where PR 8 lost 6;
  - made alone, on the five largest maps: 172 of 35,066 frames late, and mapping's load 1.7 to
    2.6. ORB-SLAM's mapping thread refuses a keyframe while busy and cuts its local BA short,
    which the inline pipeline never does.
- **Fusion's neighbours:** `mapping.fuse_neighbours` (10; `mapping.mono_fuse_neighbours`, 20)
  and `mapping.fuse_second_neighbours` (5), ORB-SLAM2's numbers in ORB-SLAM3's order.

### Changed

- **The local map, local BA, culling, triangulation and fusion take the graph's keyframes,**
  the window's no longer (`mapping.window` is gone); the keyframe decision counts against the
  frame's reference keyframe, not the newest; the local fallback searches the last tracked
  frame's local map; relocalisation's candidate groups take each candidate's best links
  (`edge.database.best_covisible`, B1's stand-in, is gone).
- **The settings are grouped by stage** (features, matching, stereo, initialization, tracking,
  adjustment, keyframes, mapping, covisibility, relocalization, loss, and the seed), each named
  `group.field` on a command line (`--option relocalization.enabled=false`), in an error and in
  a run's record, which nests them by group: 97 in all.
- **The ladder:** L15 closed; PR 9's item on mapping in a thread of its own, left for milestone
  7, is decided in B3's design.
- **The comments, docstrings and errors** name every setting `group.field`, as the code does
  (the vocabulary's errors say `relocalization.enabled`), and none describes the window.
- **Culling keyframes refuses a map without any** in the three languages, where it did nothing
  before; the viewer refuses a bundle whose graph names a keyframe the run has not.
- **The golden record** is regenerated: every room keeps its states, the frames tracked and
  lost; 12 cases make more keyframes. It gains each keyframe's last parent and the strong links.

### Tests

- The covisibility graph's fixture (`tests/fixtures/edge/covisibility.json`), by brute force:
  5 cases, ties, the kept best link, a tree repaired in rounds, the local map at its cap, and
  what fusion and local BA take, each of fusion's counts binding.
- The room's last map, in each mode, and every map of the golden record are one tree, each
  chain of parents reaching the first keyframe; the run format's writers and readers refuse a
  bad parent or link in the three languages.
- B2's seven settings moved off their defaults in the gap room's whole runs, where the natives
  must decide as Python does; the keyframe decision decides both ways for one frame against two
  reference keyframes; a culled keyframe's children, culling an empty map, a bad parent,
  reference keyframe or point id, each refused or held in the three languages.

## [0.3.10.0] - 2026-10-16

Milestone 2B's first part, B1: place recognition and relocalisation
(`docs/design/2026-10-14-online-slam.md`, section 3). A frame that tracking fails is relocalised
against the whole active map through a bag of words, as ORB-SLAM's is, and a lost track is given
3 s to be found again before its map closes, as visual-only ORB-SLAM3's recently lost track is.
The three languages relocalise alike. On the test set at seed 0, 31 runs relocalise 123 frames,
all but one where the ground truth puts them, and maps fall from 359 to 268 and losses from 348
to 231; but 6.4% fewer frames are tracked, the 3 s waited through costing more than it finds.
The window is kept for B1 and measured again with B4's merging. The run format is version 5:
runs written by 0.3.9.0 no longer read.

### Added

- **Place recognition's vocabulary** (`slam_it.foundations.vocabulary`, with C++ and Rust
  twins): DBoW2's tree of binary words over our own ORB descriptors (k = 10), a descriptor's
  word, a frame's weighted vector and DBoW2's L1 score, alike in the three languages to the bit,
  held to a parity fixture. **Its folder** (`slam_it.eval.vocabularies`, with readers in
  `slam_it_eval` and `slam-it-eval`): five `.npy` arrays, a header array of its format and
  descriptor versions, which every reader checks, and the arrays' SHA-256, which Python checks
  before any program runs (the native programs trust them, the ladder's one exception).
- **The training tool** (`scripts/train_vocabulary.py`): hierarchical k-means from a fixed seed
  on frames drawn from Bovisa or ARKitScenes, the same tree on a rebuild (its weights to their
  rule: numpy's log may differ in the last bit on another CPU). **The default
  vocabulary**, committed: 5 levels and 99,961 words, from 10,000 frames of ARKitScenes'
  training fold. Two of 6 levels, from Bovisa and ARKitScenes, are kept outside the repository
  and named, with the commands that rebuild them, in `slam_it/eval/vocabularies/local.json`.
- **Matching by word** (`matching.search_by_bow`, three languages): ORB-SLAM's SearchByBoW.
- **The keyframe database and relocalisation** (`edge.database`, `edge.relocalization`, three
  languages): ORB-SLAM's KeyFrameDatabase and `Tracking::Relocalization`, with its thresholds and
  MLPnP's RANSAC budget; candidates tried best first.
- **Relocalisation in the pipelines:**
  - a frame the motion model and the local fallback fail is relocalised against the whole map,
    and the track is lost once `relocalize_seconds` (3) have passed since its first failed
    frame with none tracked; a map of 10 keyframes or fewer is lost after 5 failures, as before;
  - `Options.relocalize`, on; off, nothing relocalises and a map is lost as A2 loses it;
  - the vocabulary, required with relocalisation on (`--vocabulary`, the committed one by
    default), refused with it off, and recorded in `run.json`'s `vocabulary`, its header and
    checksums, which every language writes from what it read (`Run.vocabulary`; run format
    version 5);
  - each frame's diagnostics: `placed_by` "relocalization", and relocalisation's summary (its
    words, the keyframes sharing a word and scored, each candidate as far as it got).
- **The found room** in the golden record: blank frames, then the start seen again, which every
  backend, in every case, relocalises in map 0 on frame 34; and a golden bundle of it, which the
  viewer's tests read. **The window room**: found again, then lost by `relocalize_seconds`
  (0.05 s there) before `lost_frames`, an RGB-D run culling keyframes first, with
  relocalisation's settings off their defaults.
- **Native tests** of the vocabulary, its reader, the keyframe database, relocalisation and the
  pipeline relocalising, in C++ and Rust; matching by word's fixture cases run natively; and
  training's rules held to known answers.
- **Relocalisation counted and shown:** `runs.tracking`'s `relocalizations` and the measure's
  "relocalised" column; the status page says how many frames tracking failed and found again;
  the viewer marks each such frame on its timeline and shows the Relocalisation stage in the
  solver panel.
- **The measurement scripts:** `scripts/measure_testset.py --option KEY=VALUE`, a setting
  replaced in every run; `scripts/measure_sharded.py`, the test set side by side, a worker a CPU,
  at several seeds; `scripts/compare_testsets.py`, two measures compared: by group, the runs that
  moved, how failures ended, latency by frame, relocalised frames against the ground truth.
- **The test set's B1 results**, in `docs/notes/test-set.md`:
  - relocalisation finds 123 frames again in 31 runs, all but one within three times their map's
    usual error;
  - maps from 359 to 268, losses from 348 to 231, and 6.4% fewer frames tracked: 231 stretches
    still end lost after waiting 3 s, 17,439 frames past A2's 5 failures;
  - ZJU B5's four black stretches and two of B7's three are found again in map 0;
  - a relocalised frame takes a median of 14.3 to 15.4 ms against a deadline of 32 to 33, none
    late, made alone.
- **"Whole truth"** in the viewer's map panel: the ground truth drawn whole.

### Changed

- **A monocular start's two-view RANSAC draws at most 200 samples a model**
  (`init_ransac_iterations`), as ORB-SLAM's initialiser does. No decision of the test set's
  monocular runs moves at seeds 0, 1 and 2; on the 7 ADVIO runs late in A2, made alone, the late
  frames fall from 108 to 0 and the slowest from 442.7 to 25.4 ms.
- **The viewer's ground truth** is faint gold, drawn over the frames the shown map's trajectory
  covers so far.
- **The run format, version 5:** `run.json`'s `vocabulary` is required, null or the header and
  checksums of the vocabulary the run relocalised with.
- **One Hamming distance a language**, in `orb` (`orb.distance`, `orb::distance`), which
  matching, stereo, the vocabulary and the edge rung call; numpy's floor is 2.0, for
  `np.bitwise_count`.
- **The keyframe database sits beside bundle adjustment** in the edge rung's import order.

### Fixed

- **The viewer's Solver panel** keeps 260 px on a wide or short window, which had left it none;
  the frame shrinks, its proportions kept.
- **C++'s vocabulary check** no longer reads out of range on a malformed tree (a parent after its
  child, children past the int32 range, a leaf's first child), and the reader no longer throws
  on a shape too large to hold.
- **The keyframe database** refuses a vector whose words are not integers, as the vocabulary
  does, where Python truncated them.
- **`scripts/measure_sharded.py`** writes each result as its run ends, and records a failed
  worker's run, where one failure lost every result.

## [0.3.9.0] - 2026-10-14

Milestone 2A's second part, A2: a lost track no longer ends a run. Its map closes, kept or, if it
has 10 keyframes or fewer, reset as ORB-SLAM3 resets its active map, and the next frame starts a
new one, so a run goes on to the sequence's end and tracks it in segments, one per map
(`docs/design/2026-10-12-phone-robust-vo.md`, section 5). All three languages decide alike across
a new map. Relocalising against a lost map, and joining maps, are milestone 2B's. A run or bundle
written before this version no longer reads. On the test set, 87% of the frames are tracked,
against 31% when a run ended at its first loss, and every frame run before is decided as
before.

### Added

- **A new map after a lost track** (`slam_it.edge.pipeline`, and the C++ and Rust pipelines):
  - after `lost_frames` failures in a row the map closes; one of `reset_keyframes` (10)
    keyframes or fewer is reset, its keyframes and points removed and its frames' poses
    dropped; a larger one is kept;
  - the next frame starts a new map, numbered after the last; each map numbers its own
    keyframes and points from 0, and the run's ids continue across maps;
  - the lost frame's diagnostics say how many keyframes and points the map had, and whether it
    was reset.
- **Run format version 4:** each frame's diagnostic names its map, and `pose_maps.npy`,
  `keyframe_maps.npy` and `point_maps.npy` give each pose's, keyframe's and point's; a removal
  `reset`. `trajectory.txt` puts a `# map <n>` line before each map's first pose, so an outside
  tool aligns one map at a time, and the writers in all three languages, and `read_run`, refuse
  frames' maps that are not null or map numbers, descend, or leave a map with keyframes unnamed.
  `Run.maps()` and `Run.reset_maps()` name a run's maps and those reset.
- **Scoring by map:** `runs.evaluate` and `evaluate_absolute` align each map on its own and pool
  the ATE (and the RPE, where a map has pairs) over them; `runs.tracking` gives a segment per
  map, the losses and the resets. `metrics.interval_pairs` gives the RPE's pairs alone.
- **The lost room:** the room with two stretches of frames blanked to lose the track twice, its
  lost maps reset and kept, in the golden record that holds the three languages to one another;
  a stereo or RGB-D run reaches a third map.
- **The test set's A2 results**, in `docs/notes/test-set.md`, at seeds 0, 1 and 2:
  - a run's median share tracked from 20% to 90%, and the seed's sway on it from 76 points at
    most to 13;
  - 348 losses over 61 runs at seed 0, 55 maps reset; a kept map spans a median of 4.5 to
    24.6 s by dataset and mode, each in its own frame;
  - the check that failed at each loss: over half are placed frames whose local map kept too
    few inliers;
  - real time breaks only while a monocular map starts again on ADVIO: 105 frames of failed
    two-view starts, up to 0.43 s.

### Changed

- **The bundle, version 2:** each map moved onto the truth by its own alignment, the maps'
  arrays, and a list of each backend's maps with their scores. The viewer shows the map of the
  frame shown, splits the timeline by map, draws a reset map's frames broken, and shows the
  maps in the header.
- **The demo's summary and status page** give the stretches tracked ("Tracked 2.9–15.9 s and
  17.0–27.1 s of your 27.1 s clip."), the losses and the maps reset.
- **`scripts/measure_testset.py`** reports each run's segments, losses and resets, and each
  map's ATE beside the pooled one.
- **A depth start is ORB-SLAM's:** a frame of more than 500 keypoints starts a map however few
  have depth; `min_init_depth_points` and the outcome `too_little_depth` are gone (open item
  L13).
- **The bake places a run by its ATE alone**, its RPE null where its reference has no pair a
  frame period apart (A1's deferred A10).
- **A run that cannot be placed is `Degenerate`:** `evaluate` and `evaluate_absolute` raise
  `DegenerateError`, not `InvalidInputError`, when no map can be aligned, naming each map's
  cause, and `metrics.relative_error` when no pose has another the interval later; an
  `rpe_delta` that is not positive and finite is refused before any map is scored.

## [0.3.8.0] - 2026-10-13

Milestone 2A's first part, A1: a test set to measure phone-robust VO against, and phone video
imported better (`docs/design/2026-10-12-phone-robust-vo.md`, section 4). The test set is public
datasets with ground truth or a reference, so the owner's own recordings stay a check on the
demo rather than the yardstick:
- TUM RGB-D's hard sequences;
- ZJU-SenseTime's phones;
- ADVIO's iPhone;
- ARKitScenes' iPad with LiDAR.

A Stray Scanner recording with LiDAR depth now runs RGB-D in the demo, and features follow the
frame size. Nothing changes for a dataset run at the default settings. The test set's results
are in `docs/notes/test-set.md`. The review panel and a security pass read it all
(`docs/reviews/2026-10-13-feature-2a-a1-measure-and-import*.md`).

### Added

- **Dataset imports to TUM format**, each with its ground truth moved to the camera:
  - `slam_it.eval.zju`: ZJU-SenseTime's sequences, frames linked, the camera radtan when it is
    distorted;
  - `slam_it.eval.advio`: ADVIO's iPhone video, through the video import with a dataset's
    limits. Frames are timed by `frames.csv` and kept distorted with the published Kalibr
    calibration of their capture batch. The poses, in the phone's IMU axes, are moved by
    `T_cam_imu`;
  - `slam_it.eval.arkitscenes`: ARKitScenes' raw captures as RGB-D. ARKit's poses are
    inverted, interpolated at each frame's time, and named a reference, not truth. Its 10 Hz
    poses fall between the 30 Hz frames, and paired by time each would serve two frames.
- **LiDAR depth as RGB-D** (`slam_it.eval.lidar`): depth kept where ARKit's confidence is high
  (medium on request), brought to the colour frame by nearest neighbour, with a virtual baseline
  of 0.08 m (ORB-SLAM's for TUM's Kinect).
- **A Stray Scanner zip's depth:** its `depth/` and `confidence/` maps are read for each frame
  kept, decoded from memory, each map's size checked as Pillow will decode it before any pixel
  is. Every kept frame needs both. The import writes an RGB-D folder, and the demo's worker runs
  it RGB-D.
- **Features by frame size** (`slam_it.edge.options.features_for`): ORB-SLAM's 1,000 at 640x480,
  scaled with the frame's area within 1,000 to 2,000. The demo's worker and the measure both set
  a run up with it (`slam_it.orchestration.vo.options_for`); the default is unchanged.
- **The video import for datasets:** `Limits` (the longest video, its most frames, the short
  side), a dataset's clock for each frame's time, its ground truth written with the frames, and
  `lenses.RadTan`, a radial-tangential lens kept and turned with its frames.
- **Every import to TUM format staged and recorded** (`slam_it.eval.tum.staged`,
  `tum.write_source`): written beside its folder and moved whole, so a failed import leaves
  nothing, with a `source.json` naming what made it (and, with LiDAR depth, the least
  confidence kept).
- **Scoring:** `slam_it.eval.runs.evaluate_absolute`, the ATE alone, which the measure reports
  and which needs no pair of frames a period apart; `runs.tracking`, what of a sequence a run
  ran and tracked, which the demo's summary and the measure share.
- **Images from bytes** (`slam_it.eval.images`): the readers take a file's bytes and a longest
  side, checked once Pillow has parsed every header; `images.size` reads the size alone.
  Pillow's every parse error, and its decompression-bomb warning, are invalid input.
- **`scripts/measure_testset.py`:** each sequence of a config run and measured:
  - the share tracked;
  - when tracking started and stopped, and whether it was lost;
  - the ATE, naming its reference;
  - the 95th-percentile latency, against the frame period.

  `--camera nominal` measures what a guessed lens costs. The config, holding a machine's paths,
  stays outside the repository; `scripts/testset.template.toml` is its format.
- **The test set's results**, `docs/notes/test-set.md`, from the public datasets alone, each
  change read against three RANSAC seeds:
  - 61 of the 77 runs lose the track, and 65% of the set's frames come after a loss, never run:
    A2's case;
  - a guessed lens costs accuracy in step with its error, most in RGB-D, but not the track;
  - ADVIO at a 720-pixel short side tracks no longer and nearly doubles a frame's latency, so
    480 stays;
  - LiDAR depth of medium confidence changes neither the track nor the error, so high stays the
    default.

### Changed

- **The demo's summary** names the run's mode and the frames' size. The status page shows a mode
  row, explains an RGB-D run's first map by its depth, and warns, never refuses, when a video
  arrived under 480 pixels on its short side.
- **The upload page** says a video runs monocular and a Stray Scanner recording RGB-D at true
  scale, and advises recording with the Camera app rather than through the browser.
- **`source.json`** records the depth imported and a kept lens's distortion.
- **The worker cuts a failure's reason** to the 500 characters the server keeps, so no upload
  can make the server refuse the report.
- **EuRoC's `sensor.yaml` helpers** are lifted into `slam_it.eval.sensor_yaml`, for ZJU-SenseTime
  and ADVIO too.

## [0.3.7.0] - 2026-10-12

The project is renamed from SLAM It All! to SLAM It!, which is shorter, easier to remember, and what
the demo does to a video. The rename is a clean break with no alias. Nothing computes differently:
the room's runs in all three languages still match their golden record, which the rename did not
touch, and runs before and after on TUM fr1/desk (RGB-D, 133 frames, and monocular, 150) and EuRoC
MH_01 (stereo, 200) in all three languages are identical file for file, features and lifetimes
included, but for the measured timings and peak memory and `run.json`'s format name. Every name
differs, though: the formats', the environment variables' (a deployment sets the new `SLAM_IT_*`
ones), the modules' and crates', the images', and the demo's default folder's (one of the old name
is not read).

### Changed

- **Code uses `slam_it`:**
  - the Python package, and the server's, `slam_it_server`;
  - the C++ namespace and include directory;
  - the libraries and programs (`slam_it_foundations`, `slam_it_vo`);
  - the extension modules `slam_it._cpp` and `slam_it_rs`.
- **Everything else uses `slam-it`:**
  - the distribution;
  - the Rust crates (`slam-it-foundations` and the rest);
  - the conda envs (`slam-it`, `slam-it-ci`) and their lock (`scripts/locks/slam-it.lock`);
  - the review panel (`/slam-it-review-panel`, `slam-it-*-reviewer`);
  - the Docker images (`slam-it-server`, `slam-it-worker`), the viewer's package, the demo's
    folder (`~/.local/share/slam-it/demo`) and the planned subdomain
    (`slam-it.timedilated.com`).
- **Other forms:**
  - `SLAM_IT_` prefixes the CMake options and environment variables, the server's and the
    worker's settings among them (`SLAM_IT_INVITE_CODES`, `SLAM_IT_WORKER_TOKEN`);
  - `SlamItError` is the base exception.
- **Renamed files:** `scripts/setup_slam_it.sh` builds the env, and the ladder is
  `docs/design/2026-10-02-slam-it-ladder.md`.
- **The formats are named `slam-it-run` and `slam-it-bundle`.** A run or bundle written before
  the rename no longer reads.
- **History is kept as written.** Earlier entries here and the review reports in
  `docs/reviews/` keep the old names.

## [0.3.6.0] - 2026-10-11

Your own video: a phone video, or a Stray Scanner recording, uploaded through a page, imported
with the best intrinsics to be had, run through the C++ pipeline by a worker, and opened in the
run viewer. The server that holds the storage keys never parses a video; an upload is kept until
its import ends, and a result for 7 days. Milestone 2 closes with it. The import and the
intrinsics are tested on synthetic recordings of the room; real phone clips will check what
`docs/notes/your-own-video.md` lists. The review panel and a security pass read it all, and
what they changed is in the entries below (`docs/reviews/2026-10-11-feature-your-own-video*.md`).

### Added

- **Video import**, `python -m slam_it_all.eval.video` (`import_video`; PyAV, in the optional
  `video` extra and kept in this module by a contract): a video opened as MP4 or MOV alone, turned
  upright by the rotation it declares, timed by presentation time (60 fps becomes 30), scaled down
  to a 480-pixel short side, undistorted to pinhole if its lens is a fisheye, and written as a
  TUM-format folder (`eval.tum.write_list`, `write_camera`) with `source.json`, which keeps at most
  four metadata fields and never a location. Frames over 7680 pixels long or 3 to 1, frames that
  change size, and videos over 14,400 frames, are refused.
- **Stray Scanner recordings** (`import_stray`): the zip read in place, three files opened, each
  capped, unsafe or unreadable entries refusing it; its poses, already in our camera's axes as
  its encoder writes them, become the ground truth, and its video's first frame, behind an edit
  list, is decoded.
- **A video's intrinsics** (`slam_it_all.eval.lenses`): a Gyroflow lens profile (CC0, fetched at a
  pinned commit by `python -m slam_it_all.eval.lenses fetch`, never committed; 1,047 of its 1,164
  phone profiles usable, the database required) matched by phone, lens (by its name, else its
  35mm-equivalent focal length), the profile's own focal length against the video's, aspect and
  orientation; else the 35mm-equivalent focal length; else a nominal 72.37-degree diagonal field of
  view. Fisheye undistortion through OpenCV's model, held to `cv2.fisheye`, resampled by rung 0's
  remap. `eval.images.write_png`.
- **The worker** (`python -m slam_it_all.orchestration.worker serve`): claims the server's jobs over
  HTTP with its token, and in a child process under memory and file-size limits and a timeout
  imports, runs and bakes each; it uploads the bundle and reports the job done, or failed with a
  reason fit to show. It repeats its stage every 30 s as the job's heartbeat, transfers included,
  and carries on past a report that does not arrive. A job's processes are one group, killed whole
  when it ends, however the worker is stopped; the child inherits no token; only `http(s)` links to
  safe hosts are followed (the host judged as urllib connects to it), and no redirect but a
  download's to one; the unexpected is logged by job. An upload's run is named `your-video`.
- **The upload server** (`server/`, `slam_it_all_server`, `python -m slam_it_all_server`): the job
  list in SQLite, which keeps its format and refuses another; storage in an S3 bucket or a folder,
  through links signed for one key and, for uploads, one size and 15 minutes; the guards (an invite
  code, at most 2 jobs open per code and 5 in all, the caps, one job worked on at a time, a
  confirmed upload moved out of its link's reach and deleted after import (kept with its result in
  the demo, `SLAM_IT_ALL_KEEP_UPLOADS`, which only folder storage takes, a new job then refused
  under 5 GB free), a job's files in one folder of storage, deleted whole with the job, what a job's
  links write later deleted again once they have all expired and an hour more has passed, a result's
  files only a bundle's, results after 7 days, a job's files deleted before it is and retried if
  that fails, tokens unguessable, pages `noindex` under a Content Security Policy that allows
  three.js's own path alone of its CDN, staging behind a code, never run as root). It logs jobs by
  id, never by token. Its settings come from the environment, listed in
  `server/settings.template.env`. Given `::`, as in its image, it listens on IPv6 and IPv4 at once;
  it stops within 3 s. It never imports `slam_it_all`.
- **The upload and status pages** (`viewer/upload.html`, `viewer/status.html`, their logic in
  `viewer/src/jobs.js`). The status page says what of the clip was tracked and where the run
  ended, by losing the track or not, from the worker's summary (`worker.summarize`), so that a
  result shorter than the video is not taken for a cropped one.
- **The images** (`docker/server.Dockerfile`, which drops root at once for good, and
  `docker/worker.Dockerfile`, with tini) and **`scripts/serve_demo.py`**, which runs the server and
  a worker on this machine, on the local network with `--lan`, every page then behind the code and
  the page's address printed as a QR code for a phone's camera (segno, in the `dev` extra and the
  lock). Its invite code is three short words unless given; its folder is the same every run,
  `~/.local/share/slam-it-all/demo` unless given, readable by its owner alone.
- **A design note**, `docs/notes/your-own-video.md`, with the staging and demo checklists.

### Changed

- **The viewer shows each backend's frames and the whole sequence:** under the frame slider, a
  row per backend over the frames it ran, coloured by state (no map yet, tracking, lost), with a
  legend; beside the slider, the parts of the sequence no run reached, as wide as their time,
  "not processed: tracking was lost" where the runs ended by losing the track. A backend's
  button is disabled on frames it did not run, and the page switches to one that did.
- **A bundle records its sequence's extent** (`bundle.json`'s `extent`: its frames, and its
  first and last frames' times since the bundle's first), which the page requires; the format
  stays version 1, and a bundle baked before is refused until baked again.
- **`Pinhole` moves to `eval.lenses`.**
- **Layer 1 covers the server:** ruff, mypy and jscpd check `server/`, prettier the new pages, and
  import contracts keep the server and `slam_it_all` apart, keep a decoder and reifyscene out of the
  server and `eval.video` out of orchestration but the worker, and order the server's modules.
- **The run viewer** reads a bundle from its own site only, and loads three.js pinned by its
  hashes.
- **The setup script** installs the `video` extra and the server, and fetches the lens profiles;
  the env's lock adds PyAV, FastAPI, uvicorn, boto3 and httpx. It pins Rust to 1.99.0, as it
  pins clang: an unpinned release added a clippy lint that failed a fresh env, and three
  asserts now meet it.
- **The README and the ladder's status** describe milestone 2 whole; the README lists
  `viewer/`, `server/` and `docker/`.

## [0.3.5.0] - 2026-10-10

The run viewer: a run records how every solver went, when each point and keyframe was made
and removed, and, if asked, what became of every keypoint; the bake packages runs of a
sequence with its images; and a page shows them frame by frame, for each backend side by
side. Nothing a run decides moves: Rust's and Python's runs match the release before byte for
byte in every file they had, and C++'s differ only in their last bits, since C++ runs now
repeat bit for bit on either core type of a hybrid CPU; the three languages agree exactly on
everything new. With features recorded, every run stays real time; the lifetimes cost up to
26 MiB of peak memory.

### Added

- **Per-frame solver summaries** in every backend's diagnostics (`FrameSummary`): how the
  frame was placed, and initialisation, tracking the last frame, the PnP fallback, tracking
  the local map and local mapping, each an object or null, bundle adjustments with each solve's
  costs, iterations and termination. Each stage returns its summary with its result
  (`Motion`, `Relocalization`, `LocalTracking`, `Initialization`, `Mapped`, `Adjustment`).
- **Lifetimes**, run format version 3: the map keeps what it removes aside (`gone_*`, with
  `Removal`: culled, fused, weak), the pipeline dates each removal (`Pipeline.lifetimes`), and
  a run holds every point and keyframe it made with its life (`point_lives.npy`,
  `keyframe_lives.npy`); `Run.final_points()` and `final_keyframes()` give the final map.
- **Feature records**, opt-in (`--record-features`, `vo.run(..., record_features=True)`):
  each frame's keypoints, their pixel, level, fate (`Fate`: unmatched, rejected, tracked,
  made) and point, streamed by `runs.FeatureWriter` in all three languages to
  `feature_pixels.npy` and `feature_records.npy`; `runs.read_features` maps them.
- **The bake**, `python -m slam_it_all.orchestration.bundles`: runs of a sequence, one per
  backend, with their frames as JPEG (`images.encode_jpeg`) into a bundle, each run aligned to
  the ground truth, or left in its own frame if its alignment cannot place it; written beside
  the bundle's folder and moved onto it whole. `vo.open_source` opens a sequence as a run reads
  it; `runs.ground_truth` reads a sequence's ground truth (for the bake and `run_vo.py`), and
  `AbsoluteError.similarity` says which alignment was fitted.
- **The run viewer** (`viewer/`): the frame with its keypoints' fates, the map as it was after
  the frame with every backend's trajectory and the truth, the solvers' summaries, plots and
  step times, the frames where the backends decided differently marked;
  `scripts/view_bundle.py` serves it with a bundle locally.
- **Layer 1's JavaScript checks:** `node --check`, `node --test`, `tsc --checkJs` (the page,
  and its logic modules again without the DOM) and prettier, with TypeScript, three.js's types
  and prettier pinned in `viewer/package-lock.json`; jscpd covers the page. An import-linter
  contract keeps `orchestration.vo` below the bake.
- **A design note**, `docs/notes/viewer.md`.

### Changed

- **Run format version 3:** `points.npy`, `keyframes.npy` and `keyframe_poses.npy` hold every
  point and keyframe the run made, by id; `run.json` gains `record_features`. Version 2 runs no
  longer read.
- **`mapping.process` returns a `Mapped`** in C++ and Rust too (the summary and the points
  made); each tracking stage returns its matches before refinement (`matched`).
- **The programs read `--record-features`**, a flag without a value, in all three languages.
- **`runs.DATASETS` replaces `vo.DATASETS`:** eval, which reads the datasets, holds their list;
  `vo.open_source` refuses a dataset it does not know.
- **The import contracts inside packages are exhaustive** (rung 0's, bundle adjustment's, the
  edge rung's, eval's and orchestration's orders): a new module fails until it is given a layer.
- **The room's TUM folder holds its ground truth** (`groundtruth.txt`).

### Fixed

- **C++ runs repeat on every core.** Eigen blocked a large product by the cache sizes of the
  core that ran the first one, which differ between a hybrid CPU's core types, so a run's last
  bits depended on where it started; every rung now compiles with `EIGEN_NO_CPUID`, so Eigen
  blocks by its own defaults on every core, in every program, test and the extension.

## [0.3.4.0] - 2026-10-08

The project is renamed SLAM It All! (a pun on "damn it all!") before it goes public. The
working name, slamlab, collided with SLAMLab, the University of Zaragoza group behind
ORB-SLAM3, and with `slamlab` on PyPI. The rename is a clean break with no alias. Nothing
behaves differently: every run checked takes the same decisions and writes the same
trajectory, keyframes and map, to the byte, as before the rename; only `run.json`'s format
name differs.

### Changed

- **Code uses `slam_it_all`:**
  - the Python package;
  - the C++ namespace and include directory;
  - the libraries and programs (`slam_it_all_foundations`, `slam_it_all_vo`);
  - the extension modules `slam_it_all._cpp` and `slam_it_all_rs`.
- **Everything else uses `slam-it-all`:**
  - the distribution;
  - the Rust crates (`slam-it-all-foundations` and the rest);
  - the conda envs (`slam-it-all`, `slam-it-all-ci`) and their lock
    (`scripts/locks/slam-it-all.lock`);
  - the review panel (`/slam-it-all-review-panel`, `slam-it-all-*-reviewer`).
- **Other forms:**
  - `SLAM_IT_ALL_` prefixes the CMake options and environment variables
    (`SLAM_IT_ALL_BUILD_TESTS`, `SLAM_IT_ALL_BAL`, `SLAM_IT_ALL_REQUIRE_BACKENDS`);
  - `SlamItAllError` is the base exception.
- **Renamed files:** `scripts/setup_slam_it_all.sh` builds the env, and the ladder is
  `docs/design/2026-10-02-slam-it-all-ladder.md`.
- **The run format is named `slam-it-all-run`.** A run written before the rename no longer
  reads.
- **History is kept as written.** Earlier entries here and the review reports in
  `docs/reviews/` keep the old name.

## [0.3.3.0] - 2026-10-08

Milestone 2's real time: a budget harness, and the C++ and Rust pipelines within a laptop-CPU
budget (4 performance cores, 8 GiB) on every sequence of the edge note's set (five TUM and
EuRoC sequences, ten runs of a sequence in a mode), no frame late. Building and tracking a
frame takes a median of 9 to 13 ms in either language on TUM and EuRoC monocular, and 21 to 26
ms on EuRoC stereo, against deadlines of 32 and 50 ms; before, 23 to 72 ms in C++ and 34 to
113 ms in Rust. Mapping, judged as a thread of its own, keeps a load of at most 0.37 over a
run and 0.92 over its busiest 10 seconds (2.28 before). Frame building stays exact; local BA
is restructured, no longer the same to the bit, and every whole run still takes PR 8's
decisions on every frame, in both languages and from all five starts, so the edge note's
medians stand.

### Added

- **The budget harness** (`slamlab.eval.budgets`): `laptop-cpu`, 4 cores and 8 GiB, its
  cores the highest-numbered performance cores the process may use, a `Budget` checking itself
  when made; a program run within it through `systemd-run --scope` (memory capped, no swap, so
  going over kills it) and `taskset`; `judge`, the frames against their deadline (the median
  frame spacing): building plus tracking (median, 95th percentile, maximum, late frames),
  reading apart, and mapping's time per keyframe, its load over the run and over its busiest
  10 seconds (or the whole run, if shorter); a run it cannot judge (times that do not advance,
  a time or step time that is not a number, negative or not finite) refused.
- **Each frame's steps timed**, in every backend's diagnostics: `read_seconds`,
  `build_seconds`, `track_seconds` and `map_seconds`, named by the run format
  (`runs.STEPS`).
- **A run records the process that ran it** (`run.json`, the run format's version 2): its
  `peak_memory` (`VmHWM`), its `cpus` (its affinity) and its `memory_cap` (its control group's
  `memory.max`), each read by the program before it writes the run (`budgets.peak_memory`,
  `allowed_cpus`, `memory_cap` in all three languages; `eval::budgets`,
  `slamlab_eval::budgets`); null for an in-process run, or a file the program cannot read.
- **Python's program**, `python -m slamlab.orchestration.vo`, with the native programs'
  command line, exit codes and messages (`vo.parse`, `vo.parse_setting`); `vo.run(...,
  budget=...)` runs any backend's program within a budget, and `scripts/run_vo.py --budget`
  prints the judgement and what the run recorded of its process (`--reuse` refuses the flags
  that would not apply).
- **Extraction from a built pyramid** (`orb.extract_levels`, `image.as_pyramid`), in all three
  languages.
- **One constructor per frame** (`edge::make_frame`, `Frame::new`): every C++ and Rust frame,
  the bindings' and the tests' included, is made with its feature grid.
- **A design note**, `docs/notes/real-time.md`: the rules, the harness, where the time went,
  each speed-up, the verification and the results.

### Changed

- **`stereo.match` takes both images' pyramids** (PR 7's A1), in all three languages; the
  stereo frame builder builds each pyramid once, as ORB-SLAM's `ComputeStereoMatches` reads
  its extractor's. The C++ bindings refuse a level that is not a contiguous uint8 image, as
  Rust's do.
- **ORB extraction faster, to the bit** (frame building in half the time in C++, a quarter in
  Rust): FAST's segment test over a row at once, byte-wide and branch-free, and its scores in
  16-bit lanes; the blur's sums laid out to vectorise; the pyramid's resize in 32-bit sums.
- **Local BA 2.5 times as fast in C++, 3.8 times in Rust:** the normal equations over the
  coordinates the reduced system reads only (one triangle, mirrored), a coupling layout fixed
  per problem, fixed-size blocks by kind, their bounds taken from the camera models
  (`camera::kMaxIntrinsicsSize`). Results agree with the old code to roundoff (per solve, over
  3,136 saved problems: the same iterations, poses within 2e-14, points 1.5e-9 relative), not
  to the bit.
- **Rust's 3x3 SVD is Eigen's `JacobiSVD`**, ported: on the stack, and C++'s factors to the
  bit.
- **C++ and Rust frames hold their feature grid**, built once with the frame, as ORB-SLAM's
  `Frame` (PR 8's A11).
- **The run format is version 2**; runs of version 1 are no longer read.
- **The native programs record their process in `main`**, as Python's does; the native
  `vo::run` returns the run unwritten.
- **Layer 1's native tests run hardened** (`-D_GLIBCXX_ASSERTIONS`): libstdc++'s bounds
  checks catch an out-of-range span or access a release build would run silently. The C++
  benchmarks build apart, unhardened, in `build/bench`, the tree to time from.
- **C++ reads one sign on a number** (`eval::text::number`), as Rust and Python do: `+-5` is
  refused.
- **The design doc** records the harness as built, closes A1, A11, H19 and the 3x3 SVD, adds
  mapping's thread to milestone 7 with PR 9's panel's deferrals; the edge VO note points to
  the real-time numbers.

### Review

- The full panel (hygiene, architecture, algorithm) ran on the slice: 34 findings (3 must-fix,
  20 should-fix, 11 notes), each decided with the owner
  (`docs/reviews/2026-10-08-feature-real-time.md`). Run 2, on the fixes, all three reviewers:
  18 findings (1 must-fix, 12 should-fix, 5 notes), decided the same way.

### Tests

- **The budget tests** (`python -m pytest -m budget`, outside layer 1): the room through every
  backend's program within `laptop-cpu`, each taking the decisions it takes unbudgeted and
  recording the CPUs and memory cap it ran within, and a program over a 16 MiB budget killed.
- **The harness's refusals:** a budget of no cores or memory, times that do not advance, step
  times negative or not finite; mapping's busiest window; the process readers in all three
  languages; peak memory measured in a fresh process.
- **Every frame times its steps**, in every backend; Python's program's command line, exit
  codes and messages against the native programs', a file it cannot read or write included.
- **ORB fixture cases for `extract_levels`**, stereo matching on given pyramids, and blur
  cases for both widths of its vertical sums (12-, 13- and 15-bit kernels).
- **The 3x3 SVD's golden fixture**, written by Eigen (`foundations_test_svd3 --regenerate`)
  and held in C++ (Eigen) and Rust (`svd3`) bit for bit, signs of zero included, a symmetric
  case among its 21; its order and signs.
- **The raw pyramid bindings**, called with valid options and features, refuse a level that
  is not a contiguous uint8 image, and name a malformed one; each frame constructor builds
  the keypoints' grid and refuses a position that is not finite.
- **The `--option` readers** (`vo.parse_setting`, `run_vo.py`'s) against what the programs
  accept and refuse; the process readers on unreadable files and malformed CPU lists, in all
  three languages.
- **BA parity with fixed hosts**: anchored points hosted by fixed cameras, a pose prior on
  one; the kept-coordinate reduction (Rust native).

## [0.3.2.0] - 2026-10-06

Milestone 2's pipeline: ORB-SLAM-lite in the edge rung, monocular, stereo and RGB-D, in
Python, C++ and Rust in lock-step, and native programs that run a whole sequence without
Python. On the synthetic room the three backends make the same decisions, frame by frame.
On TUM and EuRoC with ORB-SLAM2's settings, the medians of five starts: RGB-D 1.1 to 5.4 cm
ATE over every frame, stereo 5.3 and 5.4 cm, monocular 1.1 to 11 cm, at 2 to 30 ms of
tracking and mapping a frame in C++. No RGB-D or stereo run loses the track; 6 of 20
monocular runs do, where ORB-SLAM relocalises (milestone 4). On each sequence's first 300
frames the three backends take the same decisions on every frame.

### Added

- **The edge rung** (`slamlab.edge`, `slamlab_edge`, `slamlab-edge`), each stage a function of
  explicit state with C++ and Rust ports held to the Python reference:
  - `options`: every setting named and defaulted to ORB-SLAM2's, its constants, descriptor
    distances and stereo matcher's included (76 settings), recorded in every run and each
    refused by name when out of range;
  - `frames`: one frame's features with each keypoint's bearing, depth and right column, per
    mode (RGB-D with the camera's virtual baseline, as ORB-SLAM), checked at the boundary;
  - `map`: keyframes, keypoints and points as one struct of arrays, ORB-SLAM's `MapPoint`
    bookkeeping (distinctive descriptor, normal, scale range, counts); the C++ and Rust maps
    also index each point's observations (`pt_observations`, as `MapPoint::mObservations`), so
    mapping costs what a point's observations cost, not what the whole map does;
  - `residuals`: the chi-square test every stage shares;
  - `adjust`: local BA on rung 0's solver (level weights, inverse-depth priors, Huber; two
    passes, outliers forgotten);
  - `initialization`: from depth (more than 500 keypoints, as ORB-SLAM, and 100 with depth,
    the lite stand-in for its reset of a map lost early), and monocular two-view on
    ORB-SLAM2's rules (the inliers with more than 0.36° of parallax, 100 points after BA,
    each match followed), at unit median depth;
  - `tracking`: the motion model, the local map, pose refinement weighted by level, PnP
    against the local map as the fallback;
  - `mapping`: the keyframe decision, points from depth, culling, triangulation, fusion,
    local BA over a 10-keyframe window once the map holds three, keyframe culling;
  - `pipeline`: the per-frame loop, ORB-SLAM's rules after a recovery, a failed frame ending
    the velocity, and a lost track ending the run.
- **Guided matching** (`matching.features_in_area`, `matching.search_by_projection`): ORB-SLAM's
  `GetFeaturesInArea` and `SearchByProjection` in rung 0; the C++ and Rust cores search a
  64 x 48-cell feature grid that finds exactly what Python's brute force finds. And
  `image.level_coordinate`, the full-image-to-level map.
- **What a run reads** (`eval.tum.read_frames`, `eval.euroc.read_frames`, in all three
  languages): every frame a run consumes and the camera, never the ground truth. A TUM-format
  folder may describe its own camera in `camera.txt` (model, intrinsics, size, depth scale,
  and for RGB-D the virtual baseline `bf`; TUM's freiburg cameras have ORB-SLAM2's 40).
- **The native run writer** (`eval::runs`, `slamlab_eval::runs`): the run folder written by
  hand (JSON, `.npy` byte for byte as numpy writes it), held to a golden folder.
- **The native programs** `slamlab_vo` (C++ `cpp/apps/orchestration/`, library
  `slamlab_orchestration`; Rust crate `slamlab-orchestration`): read a TUM or EuRoC sequence,
  run the pipeline, write the run folder. Found by `slamlab.backends.program`.
- **`slamlab.orchestration.vo.run`**: one interface, three backends. Python runs in-process,
  C++ and Rust run their program and read the run back; a failed run raises the same error
  kind in each.
- **`scripts/run_vo.py`**: run a sequence (or read a run), score it against the ground truth
  (ATE, RPE), and save a rerun recording.
- **The synthetic room** (`tests/edge/room.py`): a textured box room ray-cast along a known
  path, with depth and stereo views, generated at test time; every stage's parity and the
  whole-run agreement run on it, whole and with a gap of dropped frames that forces a
  recovery. Every backend's runs of it are held to a golden record
  (`tests/fixtures/edge/room_runs.json`).
- **A design note**, `docs/notes/edge-vo.md`, with the results: each whole-sequence ATE the
  median of five starts, with its range, a run that loses the track counted as a failure.

### Changed

- **`pnp.refine_pose` takes a weight per correspondence**, in all three languages, whitening
  each residual and its Jacobian by √w; the edge rung passes each level's information.
- **H-RANSAC refuses a numerically singular homography** (its smallest singular value below
  1e-4 of its largest, in the normalised frame); two-view's own rank workaround is gone.
- **`tum.Sequence.camera` replaces `Sequence.sensor`**, read from `camera.txt` or the folder
  name; `tum.read_depth` takes the depth scale.
- **The eval rung builds in the wheel too**, since the C++ program links it.
- **`backends.program` refuses a stale Rust program** (`StaleProgramError`, a
  `BackendNotBuiltError`): one older than a file it is built from, its crates read from the
  workspace's manifests, the lock included; the tests and the scripts share the check.
- **Layer 1:** `check_layers.py` holds `cpp/apps/<rung>/` to the ladder and the edge rung's
  internal order in C++ and Rust; `check.sh` builds the Rust program before pytest;
  `setup_slamlab.sh` builds it and checks that both programs are found.
- **The design doc** records the native programs, closes A9 and A10 and the non-invertible
  homography, links the edge VO note, and lists the panel's deferred items; the rig
  calibration design's export writes `bf`.

### Review

- The full panel (hygiene, architecture, algorithm) ran on the slice: 1 must-fix, 23
  should-fix and 20 notes, each decided with the owner
  (`docs/reviews/2026-10-06-feature-edge-vo.md`). Run 2, on the fixes, all three reviewers:
  no must-fix, 15 should-fix and 14 notes, decided the same way.

### Tests

- **Native tests for the frame builders** in C++ and Rust, their refusals included; the
  program's exit code checked for every error kind.
- **The native TUM readers' freiburg2 and freiburg3 calibrations** in
  `tests/fixtures/eval/sequences.json`.
- **H-RANSAC's condition margin**: 2000 minimal samples of steep planes under wide motions
  all fit, in every backend, so a threshold raised into the range real scenes reach fails.
- Native tests write to folders of their own, not fixed names under the shared temp dir, and
  Rust's remove theirs even when a test fails.
- **A gap room** (ten frames dropped, a blank frame, narrow motion windows, a low
  `max_frames`): a frame fails, the fallback recovers twice, and the keyframe hold after a
  recovery decides, every backend held to Python's decisions; again with every setting the
  review's fixes added moved off its default, which changes Python's decisions.
- **Every setting refuses a bad value by its own name**, one table-driven test per language.
- **ORB-SLAM's parallax rule tested:** views of near and far points start a map of 300 points
  where two-view's 1-degree rule keeps 200, in every backend.
- **A noisy weighted refinement case** in `pnp.json`, its minimum and cost found in 50 digits:
  ignoring the weights fails it.

## [0.3.1.1] - 2026-10-05

CI moves off GitHub Actions for PRs: their minutes are shared with other projects, and an
Actions outage held up PR 7's merge though everything had passed locally.

### Changed

- **CI runs offline** (`scripts/offline_ci.sh`): `.github/workflows/test.yml`'s steps on the
  developer's machine. A clean checkout of the committed files (merged with `develop` when
  needed), an env of its own built from the lock (`slamlab-ci`, recreated when the lock or
  setup script changes), then `scripts/check.sh`, under an 8G memory cap; each result is
  appended to `~/.cache/slamlab-offline-ci/results.tsv`. It gates every PR.
- **GitHub's Test workflow** runs on demand (`workflow_dispatch`) and on pushes to `main`
  only; the tag workflows are unchanged.
- **The PR template** records the offline CI result in place of "CI passes"; `CLAUDE.md`, the
  design doc and the review-panel skill say how CI runs.

## [0.3.1.0] - 2026-10-05

Milestone 2's first slice: the front end and the evaluation. Images, ORB features, descriptor
matching and stereo, in Python, C++ and Rust, bit-identical across the three; the datasets
the pipeline will run on, read the same way in each; and the metrics and run format that
will score it.

### Added

- **Fixed-point images** (`foundations.image`): gray conversion (OpenCV's `cvtColor`, bit for
  bit), a separable integer blur, bilinear resizing, pyramids and remapping, all in integer
  arithmetic, so the three languages' images are identical. Images cross the bindings as
  uint8.
- **ORB, as ORB-SLAM extracts it** (`foundations.orb`): FAST-9 scores (OpenCV's, exactly),
  30-pixel cells with thresholds 20 and 7, the quadtree, intensity-centroid orientation in
  30 bins, and rBRIEF with OpenCV's learned pattern (BSD, attributed). `detect`, `extract`,
  `fast_scores`, `distribute`, `level_budgets`. Keypoints carry exact full-image coordinates
  (`image.full_image_coordinate`, where ORB-SLAM's `x s^l` is up to 2 px off on a coarse
  level), and features record the pyramid they span.
- **Descriptor matching** (`foundations.matching`): Hamming distances and ORB-SLAM's filtered
  nearest neighbours (distance cap, ratio test, one match per descriptor, the
  rotation-consistency histogram).
- **Stereo** (`foundations.stereo`): Bouguet's rectification (OpenCV's rotations to 1e-15),
  rectification maps through each camera's model, and ORB-SLAM's matching along rectified
  rows, disparity and depth. The sub-pixel refinement is zero-mean SAD with a V fit, unbiased
  to 0.002 px where ORB-SLAM2's centre-subtracted SAD over-estimates disparity by up to
  0.27 px; every keypoint is checked against its features' pyramid, and a pair with the right
  camera on the left is refused.
- **Datasets** (`eval.images`, `eval.euroc`, `tum.read_depth`): image files on Pillow (in
  that one module; Pillow is now a runtime dependency), TUM depth maps in metres, and the
  EuRoC ASL format's stereo frames, calibrations and ground truth.
- **Native PNG decoders** outside the cores: the C++ `slamlab_eval` library on stb_image
  (vendored, PNG only, its symbols private to the library) and the Rust `slamlab-eval` crate
  on `png`, with Python's names (`read_gray`, `read_rgb`, `read_depth`), held with Pillow to
  seven shared PNG fixtures.
- **Trajectory metrics and the run format** (`eval.metrics`, `eval.runs`, `eval.timestamps`):
  ATE after Sim(3) or SE(3) alignment and RPE over an interval of time (a tracking gap
  skipped, not counted as one step), with timestamps paired closest first, as in the TUM
  benchmark's tools; and the run folder every backend's pipeline will write (`run.json`, a
  TUM-format trajectory, keyframes, points, diagnostics), checked whole before it is written
  and replaced file by file.
- **Native benchmarks:** `orb_extract` on one closed-form VGA frame in both suites (C++
  23 ms, Rust 35 ms).
- **`THIRD_PARTY_NOTICES`**, the design doc's file for copied material: OpenCV's ORB pattern
  and stb_image.
- **A design note**, `docs/notes/front-end.md`.

### Changed

- **The demos match slamlab's ORB features** in place of their own SIFT (closes A8), and the
  absolute-pose demo reads depth with `tum.read_depth` (closes A7). Results are comparable:
  the absolute-pose survey places 32 frames at a median 0.77 deg and 0.98 cm (SIFT: 31,
  0.76 deg, 1.11 cm).
- **The design doc** records that the front end is rung 0's (the classic rung reuses it),
  names the native PNG readers in its Languages and Package-layout tables and its error
  contract, makes exact pyramid coordinates and exact (scalar, unfused) floats conventions,
  and carries PR 7's deferred panel findings in its open items.
- **`camera.as_model`** replaces the private `_model_and_intrinsics`.
- **The fixture generators share one JSON layout** (`tests/fixtures/foundations/_layout.py`).
- **Rust's fixture tests parse floats exactly** (serde_json's `float_roundtrip`).

## [0.3.0.0] - 2026-10-04

Milestone 1 is complete: Python, C++ and Rust have the same capabilities. The Rust core
gains the estimation half of rung 0, and Rust becomes a lock-step backend. Every SVD in the
Rust core now goes through faer, since nalgebra's were not accurate on ill-conditioned input.
Porting the refinement found a crash in the C++ core, fixed here. Four deferred panel items
are closed, and this PR's own panel findings are resolved.

### Added

- **Correspondences and triangulation in Rust** (`slamlab_foundations::correspondences`,
  `triangulation`): the shared checks and normalisations, two-view triangulation (niter2)
  and the n-view DLT.
- **RANSAC in Rust** (`slamlab_foundations::ransac`): LO-MSAC over an `Estimator` trait,
  drawing the same PCG32 samples as Python and C++, and the refined run that keeps a
  refinement only if it lowers the MSAC cost.
- **Five-point, epipolar and homography in Rust:** Nistér's and Stewénius' solvers, the
  normalised 8-point E and F, Sampson errors, E's decomposition and its refinement on the
  essential manifold, the normalised DLT, Malis and Vargas' decomposition of H, and E-, F-
  and H-RANSAC.
- **Two-view initialisation in Rust** (`slamlab_foundations::twoview`): E and H by RANSAC,
  GRIC model selection and the choice of motion by triangulation.
- **P3P and PnP in Rust:** Lambda Twist and Ding et al.'s P3P, EPnP, the pose refinement on
  SE(3) and PnP RANSAC.
- **The `slamlab_rs` bindings** for every module above, with a shared RANSAC crossing that
  reads options through serde.
- **RANSAC benchmarks** in both native suites: E-RANSAC and PnP RANSAC on one fixed
  synthetic scene. Both cores keep the same inliers in the same number of iterations;
  C++ takes 2.29 and 2.40 ms, Rust 3.39 and 2.05 ms.
- **`linalg`** (Rust): faer's SVD (`svd`, `svd3`, `singular_values3`) and real
  eigendecomposition (`eigen_real`) for the whole crate, and `least_squares`, the
  minimum-norm SVD solve with an explicit cutoff, so each caller states whose convention
  (numpy's or Eigen's) it follows.

### Changed

- **Rust is a lock-step backend.** Every module's tests run on Python, C++ and Rust, and a
  compiled backend missing a module or a function fails them; the skip for a module Rust
  lacked is gone. The demos run Rust by default, and the TUM two-view and absolute-pose
  surveys print the same results on C++ and Rust.
- **The refinement callback takes the inlier indices in every language** (panel A6 on
  PR 4); Python passed a mask.
- **BA's solve has its own module** (`ba/solve.rs`, `ba/solve.cpp`), as Python has
  `ba/solve.py`, so the layer check holds it to the top of the package (panel A13, A24).
  Rust callers name `ba::solve::solve`.
- **Nesting summaries implement `solve::Summary` in Rust** by delegation: `ba::Summary` and
  `ransac::RefinedSummary` (panel A5).
- **Five-point solutions come back in one canonical order in every language** (panel L3):
  each E signed so its first entry within 1e-6 of the largest magnitude is positive (a pure
  translation's skew E ties its two largest entries), then sorted row-major. RANSAC keeps
  the first of equally good models, so the cores' decisions no longer depend on each
  library's null-space basis or Schur form.
- **The refinement callback may fail, in every language** (panel A2): a failed round is not
  kept, where the refiners used to unwrap. `start_motion` returns an error for an E that does
  not decompose, in C++ and Rust.
- **Rust's Stewénius solver** takes its eigenvectors from one eigendecomposition per sample,
  as Eigen and numpy do, and **EPnP's beta steps** use Householder QR when clearly full rank;
  together they recover most of the cost of the accurate SVD.
- **`linalg`** is crate-private and an optional layer at the bottom of rung 0's contract
  (panel A4, A8); the design doc records the cores' runtime dependencies (A7).
- **Python's `start_motion`** raises for an E that does not decompose, as the compiled cores
  return an error (A13).
- **`docs/notes/rust-port.md`** covers the estimation core: the port's choices, the SVD
  finding and the faer switch, the RANSAC benchmarks and the TUM results, and one code-size
  table for all of rung 0, counted by one stated rule.

### Fixed

- **Rust's SVDs are accurate on ill-conditioned input** (panel L1, L2). nalgebra 0.35's 3x3
  SVD squares the condition number, so Rust's Umeyama, and so P3P, returned rotations off by
  up to 0.7 on thin triangles; its dynamic SVD left the 8-point E and F up to 6e-3 from
  Python's on a few near-planar scenes. Every SVD now goes through faer, which matches
  LAPACK to roundoff on both, scaling its input into a safe range first as LAPACK does
  (faer alone gave wrong factors below about 1e-154).
- **C++ `refine_essential` no longer segfaults** on a match at both epipoles, whose Sampson
  distance is 0 / 0. `epipolar::refine` and `pnp::refine` return the solve's error and the
  public refinements raise it as `InvalidInputError`, as Python and Rust do.
- **A failed gauge SVD is `DegenerateError` in Python** too, not numpy's `LinAlgError`
  (panel A20).

### Tests

- **An epipole match raises `InvalidInputError` on every backend** (the crash above).
- **The layer check's facade cases** follow the move: the facade including a package module
  fails as `problem`, and `super::solve` from below fails in Rust.
- **`ill_conditioned.json`** (50-digit mpmath) holds every backend to near-planar 8-point
  scenes, a near-rank-deficient E and thin-triangle Umeyama, the inputs the old SVD got
  wrong.
- **The compiled-side tests run on every compiled backend** (panel A1), not only C++, and
  one test requires every module with an `Implementation` Protocol of each.
- **Native Rust tests** for each new module's main cases, the SVD's graded and thin cases,
  and the failure paths (no model from any sample, an epipole match, a rank-1 E, a point
  behind the camera); the rest of each module's behaviour is covered through the lock-step
  tests.

## [0.2.4.0] - 2026-10-04

The Rust core reaches rung 0's geometry and optimisation, at parity with C++. This is the
first of two PRs that give Python, C++ and Rust the same capabilities at the end of
milestone 1; PR 6 adds the estimation core. Two deferred panel items are closed.

### Added

- **SO(3) and SE(3) in Rust** (`slamlab_foundations::so3`, `se3`): every function, term for
  term with the C++ core, on nalgebra's fixed-size matrices.
- **Camera models and Umeyama alignment in Rust**: pinhole, radtan and radial, with the
  projection Jacobian and the Newton undistortion.
- **PCG32 in Rust** (`slamlab_foundations::rng`), drawing the same samples as Python and
  C++.
- **Non-linear least squares in Rust** (`slamlab_foundations::nls`, `solve`): the generic
  Gauss-Newton and Levenberg-Marquardt loop over `Problem` and `LinearSystem` traits.
- **Bundle adjustment in Rust** (`slamlab_foundations::ba`): the gauge, layout, Schur system
  over a `ReducedSystemSolver` and the bundle's blocks. All fifteen BA parity cases match the
  Python reference, iteration counts included.
- **The `slamlab_rs` bindings** for every module above. They share one batch layer (the
  `batched!` macro), and the BA options are read through serde, with unknown fields denied at
  every depth.
- **Native benchmarks:** Google Benchmark (C++, `cpp/bench/`) and criterion (Rust) time the
  same SO(3), SE(3) and camera operations, and a whole bundle adjustment on a BAL problem.
  On ladybug-49 the C++ solve takes 1.430 s and the Rust solve 1.441 s.
- **`cargo run --example bal`:** bundle adjustment on a BAL problem from Rust alone, with no
  Python.
- **`docs/notes/rust-port.md`:** how the port went, and the C++-versus-Rust speed and size
  comparison.

### Changed

- **`demo_bal.py --backend`** takes `rust` and `all`. On ladybug-49, all three backends
  reach 0.9147 px in 33 iterations.
- **The property tests run on Rust:** every module Rust implements runs them on all three
  backends.
- **Enum names come from the cores (A27).** C++ gains core `name()` functions for the camera
  models, NLS methods and losses, and BA gauges. The BA binding's own tables are gone, and one
  `enum_by_name` reads every choice by name.
- **Options-dict errors read as serde's** (``missing field `x` ``, ``unknown field `x` ``),
  in both bindings.
- **Every compiled backend implements each module it has in full.** The Protocol test and
  `require_function` enforce it for Rust as for C++, and `backends.COMPILED_BACKENDS`
  replaces the hand-written lists (panel A1, A6).
- **Enum codes leave the Rust binding by their position in `ALL`,** the list that names and
  decodes them, and fail loudly for a value missing from it; the Python side decodes every
  code through `backends.decode_name`, which rejects a code outside its table (panel A3,
  A12).
- **`ba::Problem::with_defaults`,** in both cores, fills a problem's optional arrays as
  `BAProblem.create` does; the native BAL readers use it (panel A11).

### Fixed

- **A NaN gradient entry never reads as convergence** (panel L1). Rust's `f64::max` and
  Eigen's default `maxCoeff` dropped NaN from the gradient's norm; both cores now propagate
  it, as the reference does.
- **A non-finite matrix no longer hangs the gauge's SVD** (panel L2). nalgebra's general SVD
  has no iteration cap; one capped, finiteness-checked helper replaces it.
- **A failed SVD while planning the gauge is an error** (`Degenerate`), not a free similarity
  read as "nothing pins anything" (panel L12, Rust).
- **The native cores reject non-finite input** in the rotation, skew, transform and
  quaternion checks and in Umeyama, in both languages (panel L5, H2). NaN passes every
  tolerance comparison.

### Tests

- **PCG32's parity values move to a shared fixture (A9),**
  `tests/fixtures/foundations/rng.json`. The Python, native C++ and Rust tests all read it.
  The native C++ tests read fixtures through nlohmann_json, a test-only conda package.
- **Layer 1 builds and lints the benchmarks,** and holds `cpp/bench` to the ladder like the
  rest of the code.
- **`check_layers.py` holds Rust's modules inside a rung to the internal contracts,** reading
  their `crate::` and `super::` paths, as it does C++'s includes, and fails on what it cannot
  follow: glob imports, re-exports in a facade, test code outside the trailing test module
  (panel A2, A8). It also resolves quoted relative includes to the rung they land in (A9).
- **The native BAL readers are tested against the Python reader,** every field and default,
  through a shared fixture, `tests/fixtures/eval/bal.json` (panel A4, A10, L10).
- **Native Rust tests for bundle adjustment:** a noise-free scene recovered, degenerate
  points dropped, malformed problems rejected, and the Schur reduction (panel H1).

## [0.2.3.0] - 2026-10-03

Absolute pose, completing milestone 1's slice B: a frame placed against known 3D points, from
three correspondences to a refined, robust pose. Python reference and C++ core in lock-step,
with identical RANSAC runs on the same seed. Three deferred panel items folded in.

### Added

- **P3P** (`foundations.p3p`): Ding et al.'s solver (CVPR 2023, the default) and Lambda Twist
  (ECCV 2018) behind a `solver` option, following PoseLib's implementations; every pose
  returned is a rigid motion with every point in front. Ding's always intersects both lines of
  the degenerate conic (PoseLib's single-root shortcut dropped true poses near the danger
  cylinder).
- **EPnP** (`pnp.epnp`): Lepetit et al.'s least-squares PnP from five points, following
  OpenCV's implementation; planar points raise `Degenerate`.
- **The pose refinement** (`pnp.refine_pose`): Levenberg-Marquardt on SE(3) of the
  Cauchy-robust reprojection error, with an analytic Jacobian.
- **PnP RANSAC** (`pnp.pnp_ransac`): LO-MSAC with a P3P minimal solver and EPnP refits, the
  consensus refined and kept only if the MSAC cost drops.
- **Generic refined RANSAC** (`ransac.RefinedSummary`, `refine_consensus`,
  `estimate_refined`, in both languages), shared by E-RANSAC and PnP RANSAC: refine the
  consensus, re-collect the inliers and repeat while the MSAC cost drops
  (`REFINE_ROUNDS = 4`; `RefinedSummary.rounds`).
- **`correspondences.normalised_xy`** and **`p3p.rigid_pose`**, shared helpers.
- **`backends.call`**: one dispatch for functions whose two sides differ, decoding the compiled
  result with the module's `codes()` (PR 2 panel A8).
- **TUM depth maps**: `tum.Sequence.depths` and `tum.DEPTH_SCALE`.
- **Demo** `scripts/demo_absolute_pose.py`: PnP on TUM with points from depth or from two-view
  initialisation, against the motion capture and OpenCV, with a survey mode.
- **Parity fixtures** `p3p.json` and `pnp.json` (`generate_pose.py`), and the design note
  `docs/notes/absolute-pose.md`.

### Changed

- `epipolar.EssentialSummary` is now `ransac.RefinedSummary` (`ransac::RefinedSolution` in
  C++); `essential_summary_from_backend` is `ransac.refined_summary_from_backend`.
- E-RANSAC's refinement now runs in refine-and-reselect rounds: on the synthetic sweep the
  median error is 0.78 of the 8-point floor (0.71 with one refinement), the p90 2.94 (3.21),
  the recall 0.953 (0.950).
- `ransac.run` takes the reference, the compiled arguments and the summary decoder.
- The design doc's pose convention: rung 0's estimators return the transform they name
  (`T_cam_world`, `T_c2_c1`); trajectory results stay `T_world_cam`.
- `codes()` keys are qualified: `ransac_termination` and `nls_termination` everywhere (PR 3
  panel A7); `nls.summary_from_backend` takes the codes dict.
- BA's options cross the bindings as one nested dict of `BAOptions`' fields, choices by name,
  read strictly at every depth (PR 2 panel A7).
- `nls` sits one layer lower in rung 0's order, so `ransac` may use it.
- The bindings share one enum-code decoder, `enum_names` and the `rows3`/`rows2` helpers;
  `pnp.codes()` carries `minimal_solver`.

### Fixed

- `twoview.initialize` no longer raises when H-RANSAC returns a rank-1 homography (duplicate
  keypoints at one pixel, TUM fr1/desk): such an H is no model, and E decides.

## [0.2.2.0] - 2026-10-03

The 5-point solver and the Sampson refinement of E (milestone 1, slice B): E-RANSAC samples
five matches instead of eight and refines its consensus on the geometric error. Python
reference and C++ core in lock-step.

### Added

- **The 5-point solvers** (`foundations.fivepoint`): Stewénius' action matrix (the default)
  and Nistér's degree-10 polynomial with Sturm-sequence root isolation, behind a `solver`
  option; each solution is polished to convergence on the cubic constraints. Only essential
  solutions are returned.
- **The Sampson refinement** (`epipolar.refine_essential`): Levenberg-Marquardt on the inliers'
  robust (Cauchy) Sampson distance over E's five-dimensional manifold, with an analytic
  Jacobian, returning an `nls.Summary`.
- **E-RANSAC options**: `essential_ransac(minimal_solver=, refine=)` and
  `twoview.Options(essential_solver=, refine=)`; the demo's `--essential-solver` and
  `--no-refine`.
- **`epipolar.EssentialSummary`**: E-RANSAC's summary nests the RANSAC run's and the
  refinement's, with whether the refined E was kept; the two-view result carries it.
- **Parity fixtures** `fivepoint.json` and `refine_essential` in `epipolar.json`.
- **Design note** `docs/notes/five-point.md`, with the synthetic sweep, the TUM survey and the
  near-degenerate recall.
- PoseLib in the dev extra, as a test-only oracle; an import-linter contract keeps every test
  oracle out of the library.

### Changed

- E-RANSAC's default minimal solver is Stewénius' 5-point, and it refines its consensus by
  default (kept only if the MSAC cost drops). On TUM fr1/xyz the initialiser now initialises
  24 of 38 pairs (was 22) at a median 0.49° rotation and 2.33° translation-direction error (was
  0.88° and 4.16°). `essential_solver="8pt", refine=False` reproduces 0.2.1.0.
- `essential_ransac` returns an `EssentialSummary` instead of a `ransac.Summary`.
- A compiled backend's NLS summary crosses the bindings as one nested dict
  (`nls.summary_from_backend`), shared by BA and the refinement.
- The design doc's binding conventions say how a choice field crosses (by name inside an
  options dict, as a code on its own).

### Tests

- The two-view fallback test's desk scene is 90% planar (was 80%): the refined E erased H's
  GRIC margin on the old scene.

## [0.2.1.0] - 2026-10-02

Two-view geometry, milestone 1's slice B up to the 5-point solver: from feature matches
between two calibrated views to their relative motion and first map points, or a reason
why not. Python reference and C++ core in lock-step, with identical RANSAC runs on the
same seed.

### Added

- **The solver contract** (`foundations.solve`): the `Summary` Protocol and the generic
  `Solution[State, Summary]` every iterative solver returns. Each solver owns its summary
  (`nls.Summary`, `ransac.Summary`).
- **PCG32** (`foundations.rng`): one random generator in every backend (the reference
  library's output, an unbiased bounded draw, Floyd's sampling), so a seed draws the same
  RANSAC samples in Python and C++.
- **Triangulation** (`foundations.triangulation`): two-view optimal (Lindstrom's niter2)
  and N-view DLT on bearing vectors, with cheirality and parallax per point; parallel or
  undetermined points are NaN and counted.
- **RANSAC** (`foundations.ransac`): LO-MSAC, generic over an estimator: MSAC scoring, the
  adaptive bound, and Lebeda et al.'s local optimisation.
- **Correspondences** (`foundations.correspondences`): the unchecked building blocks the
  two-view estimators share (homogeneous and normalised points, Hartley's normalisation, the
  null vector, `PairEstimator`), with their input checks in `foundations.arrays` and
  `ransac.run`, so `epipolar` and `homography` are independent siblings.
- **Epipolar geometry** (`foundations.epipolar`): the normalised 8-point essential and
  fundamental matrices, their Sampson distances, RANSAC, and E's four-pose decomposition.
- **Homographies** (`foundations.homography`): the normalised DLT, its Sampson distance,
  RANSAC, and Malis and Vargas' decomposition.
- **Two-view initialisation** (`foundations.twoview`): E and H by RANSAC, Torr's GRIC over
  the matches either explains, the motion chosen by triangulation, and a status
  (`initialized`, `pure_rotation`, `low_parallax`, `ambiguous`) instead of an exception;
  an ambiguous H falls back to E. H-RANSAC's threshold is E's scaled for its two degrees of
  freedom.
- **`slamlab.eval.tum`**: TUM RGB-D sequences with their ground-truth poses and calibrations.
- **`scripts/demo_two_view.py`**: initialisation on a TUM pair against the ground truth and
  OpenCV. On fr1/xyz frames 0 and 30: 0.47 deg rotation and 2.4 deg translation-direction
  error (OpenCV's 5-point RANSAC: 1.30 and 5.1); 4-6 ms in C++. `--survey` runs 38 pairs.
- An import-linter contract keeps `cv2` out of slamlab.
- Design notes: `triangulation.md`, `ransac.md`, `two-view.md`.

### Changed

- NLS's `SolveSummary` is `nls.Summary`, and its `Solution` is `solve.Solution`.
- The bindings' array copy helpers moved from `ba.cpp` to `common.hpp`.
- `opencv-python-headless` joins the dev extra (the demo's feature matching only).

### Fixed

- An estimated homography's candidate rotations, and a pure rotation's, are projected onto
  SO(3); the closed form is only exact for an exact H.
- `opencv-python-headless` is in the dev extra, not only the lock.

### Review

- The full panel (hygiene, architecture, algorithm) ran on the slice: 3 must-fix, 13
  should-fix and 10 notes, each decided with the owner
  (`docs/reviews/2026-10-02-feature-two-view.md`).

### Tests

- `so3.log` at both ends of the `arccos` domain: traces a few ulps past 3 and -1.
- Fixtures from a 50-digit mpmath oracle for triangulation, the 8-point solvers, the DLT,
  Sampson distances and both decompositions, and ground-truth fixtures for RANSAC runs and
  initialisation.

## [0.2.0.0] - 2026-10-02

The bundle-adjustment spine of milestone 1: rung 0's geometry, from Lie groups to a bundle
adjuster that matches Ceres on real data. Python reference and C++ core in lock-step
throughout, each component with a design note in `docs/notes/`.

### Added

- **Lie groups** (`foundations.so3`, `.se3`, `.alignment`). SO(3) and SE(3) exp/log,
  Jacobians and their inverses, quaternions, projection onto SO(3), and Umeyama alignment
  (SE(3)/Sim(3)). Parity fixtures come from a 50-digit mpmath oracle using independent
  algorithms; SE(3)'s Q coefficients use exact power series below 1 rad.
- **Camera models** (`foundations.camera`): pinhole, radtan, and radial (COLMAP's RADIAL,
  BAL's camera, computed through radtan by a fixed linear map). Projection, Jacobians with
  respect to the point and the intrinsics, and Newton unprojection to bearing vectors.
- **Non-linear least squares on manifolds** (`foundations.nls`). Gauss-Newton and
  Levenberg-Marquardt over a `Problem`/`LinearSystem` seam; Huber and Cauchy losses by IRLS,
  with a loss per block; a full iteration trace; strict mode; a numeric-Jacobian checker.
- **Bundle adjustment** (`foundations.ba`):
  - robust reprojection over poses, intrinsics (shared or per camera, fixable per
    parameter) and points, with the points eliminated by the Schur complement behind a
    `ReducedSystemSolver` seam;
  - points as XYZ or anchored inverse depths;
  - depth priors through per-camera affines (metric or relative), and pose priors with
    decoupled centre and orientation halves (a GNSS-style centre-only prior works);
  - the gauge removed exactly: the null space of what fixed poses, fixed points, pose
    priors and metric depth pin, removed by fixing or confining as few poses as it takes
    (partial anchoring such as two control points or attitude priors included), in the
    scene's own frame, and the solve runs in a world centred on the cameras, so
    geo-referenced coordinates (up to ECEF range) solve as at the origin;
  - degenerate data dropped and counted, and constant prior blocks left out;
  - marginal deviations of free intrinsics, scaled by the a-posteriori variance factor
    (reported; IRLS-weighted, so outliers do not inflate it), with unobservable focal
    lengths flagged;
  - a parity fixture of fifteen reference solves (every gauge `auto` resolves to, every
    explicit recipe, partial ones, Gauss-Newton, robust losses, degeneracy, radial, priors
    far from the truth, anchors) that the C++ core reproduces.
- **Readers** (`slamlab.eval.readers`): BAL (plain or bz2) and COLMAP's text model.
- **`slamlab.eval`**: COLMAP's Ceres bundle adjuster as a reference, and reprojection
  metrics.
- **Demos**:
  - `scripts/demo_bal.py`: on BAL Ladybug-49 both solvers reach 0.9147 px.
  - `scripts/compare_ba_colmap.py`: slamlab against COLMAP on any COLMAP text models. On
    reifyscene's ten DAVIS slice scenes (invocation in the BA note):
    Reprojection errors agree within 0.01 px on 8 of 10. Focal length is unobservable on 9
    of 10. COLMAP lets points cross behind cameras; ours never does.
- **Dev dependencies**: pycolmap and rerun-sdk.

### Changed

- The design doc fixes the boundary conventions every binding and fixture bakes in. It also
  sets the per-PR review rule: PRs are slices, steps are commits, and the panel runs once
  per PR. Its status and open items are brought up to date, and the panel's deferrals are
  recorded.
- `scripts/check_layers.py` applies rung 0's internal order (and BA's) to C++ includes, as
  import-linter does to Python imports, chains through unlisted helpers included.
- `camera.unproject_or_nan`: per-pixel unprojection with NaN where the lens cannot be
  inverted (it was BA's private helper).
- pycolmap is pinned once, in the `colmap` extra, which the development setup installs.

### Performance

- The C++ Schur elimination does one product per point, not one per pair of its
  observations. Results are identical, and reifyscene's slice runs 2.3-5.1x faster (judo
  565 s to 111 s).

## [0.1.2.1] - 2026-10-02

### Changed

- **The review panel runs from reifyscene sessions.**
  - Work on slamlab happens from sessions started in reifyscene, so the two projects progress
    together. The panel no longer assumes a session started inside slamlab.
  - The agents are renamed `slamlab-hygiene-reviewer`, `slamlab-architecture-reviewer` and
    `slamlab-algorithm-reviewer`, and the skill `/slamlab-review-panel`, so they cannot clash
    with a future reifyscene panel.
  - Every path in them is now relative to the slamlab root (`REPO`), which the skill passes
    to each reviewer. Before, a reviewer launched from reifyscene would have read
    reifyscene's `CLAUDE.md`.
  - They are symlinked into reifyscene's git-ignored `.claude/`, so they are registered there
    and still single-sourced in slamlab.
  - `CLAUDE.md` gains a "Where sessions run" section.

## [0.1.2.0] - 2026-10-02

### Added

- **Review process in two layers.**

  **Layer 1** (`scripts/check.sh`) runs on every PR, locally and in CI. It holds all three
  languages to one standard:
  - Python: numpy-style docstrings, full type hints and `mypy --strict`, through ruff;
  - C++: clang-format, clang-tidy, and a Doxygen run that fails on any undocumented
    declaration;
  - Rust: rustfmt, and clippy pedantic with `missing_docs` on private items too;
  - all three: jscpd duplication detection and import-linter dependency contracts;
  - the C++, Rust and Python test suites, with every backend required:
    `SLAMLAB_REQUIRE_BACKENDS` turns a missing or broken extension into a failure, not a
    skipped parity test.

  CI builds the conda env with `setup_slamlab.sh` from a pinned, checksum-verified Miniforge
  (read-only token, 30-minute timeout) and runs the same script.

  **Layer 2** (`/review-panel`) runs on PATCH-or-larger changes, with three reviewer agents in
  `.claude/agents/`:
  - hygiene (Sonnet);
  - architecture (Opus);
  - algorithm (Opus, with web search and a reference list grouped by rung).

  Reports are saved in `docs/reviews/`, and each finding's resolution is decided by the owner.

- **Process documents.**
  - A PR template whose "How this was built" section records the agentic workflow.
  - A project `CLAUDE.md` with the conventions, layout, review rules and merge rule.

- **The panel reviewed this change three times** (`docs/reviews/2026-10-02-feature-review-panel.md`).
  - Run 1: 31 findings, 1 of them must-fix.
  - Run 2: 19 findings, none must-fix.
  - Run 3: all three reviewers stated the first version satisfies them, and raised 7 smaller
    items.
  - Run 4 (architecture and algorithm): run 3 resolved. 6 small items were fixed or
    recorded without a fifth run, as the owner chose.

  The owner chose to resolve every finding here. The changes below are that work.

- **C++ and Rust enforce the ladder too.**
  - One C++ library per rung (`slamlab_foundations`) and one Rust crate per rung
    (`slamlab-foundations`, previously `slamlab-core`).
  - `scripts/check_layers.py` reads import-linter's contracts from `pyproject.toml` and
    fails layer 1 on a forbidden cross-rung include, CMake link or crate dependency. It
    covers quoted and angle-bracket includes, sources outside any rung, every Cargo
    dependency table (dev, build, target), renamed and `workspace = true` crates, and
    helper crates that would relay a forbidden rung. Every crate under `rust/` is checked,
    including Cargo's implicit workspace members. There is a test for each form.
  - `check.sh` tests every rung crate: `cargo test --workspace`.

- **An error contract shared by the three languages.**
  - C++23 `std::expected<T, Error>` and Rust `Result<T, Error>`, with one list of kinds:
    `InvalidInput`, `Degenerate` and `NotConverged`.
  - Python raises the matching exception from `slamlab.foundations.errors`. `as_vec3` now
    raises `InvalidInputError`.
  - Iterative solvers will return their best estimate plus a solve summary, as in Ceres and
    SciPy. `NotConverged` is kept for strict mode or divergence (design doc).

### Changed

- **One package per rung, mirrored in every language.**
  - Python: `slamlab.foundations` (rung 0), plus stub packages for `edge`, `vi`, `classic`,
    `dense`, `learned`, `eval` and `orchestration`. Each stub's docstring states its rung,
    milestone and compute budget.
  - C++ and Rust: `cpp/src/foundations/` with `slamlab::foundations`, and
    `slamlab_core::foundations`.
  - Extension submodules: `slamlab._cpp.foundations.so3` and `slamlab_rs.foundations.so3`,
    so names never collide across rungs.
  - Import-linter contracts now cover whole packages:
    - the ladder's layers, with independent sibling branches;
    - visual-inertial builds only on edge, and learned only on dense;
    - the layers contract is exhaustive, so a new top-level package must be placed;
    - no PyTorch, GPU stack or torch-based geometry library on the CPU rungs;
    - no depth-model imports in any algorithm package;
    - slamlab never imports reifyscene.

- **Bindings take and return float64 numpy arrays** (nanobind `ndarray`, the Rust `numpy`
  crate), so batched data will cross without per-element Python objects. Both refuse
  anything else (another dtype, a strided view, a misaligned buffer) rather than silently
  converting it. They are one file
  per module, with thin roots that only wire the rungs.

- **Dispatch moved next to each algorithm.**
  - `slamlab.foundations.so3.hat(v, backend=...)` validates once, then computes with the
    Python reference or calls the chosen extension.
  - `slamlab.backends` now only locates compiled implementations. It reports an installed
    extension that fails to load with its real error, not as "not built".
  - Each algorithm module types its compiled backend with an `Implementation` Protocol. One
    generic test walks every extension submodule and checks its functions and parameter
    names against it. This replaces the mypy exemption for the extensions.

- **Stricter input validation.** `slamlab.foundations.arrays.as_vec3` rejects booleans,
  complex numbers, strings, `None` and masked arrays, as well as NaN and infinity, instead of
  coercing them.

- **Parity fixtures state their tolerance policy.**
  - `tests/conftest.assert_matches_fixture` applies the policy. An exact policy also
    compares the sign of zeros.
  - `hat`'s fixture is compared exactly, and includes values not representable in float32
    (0.1, 1/3, 1e-300), so a float32 backend now fails.
  - The native C++ and Rust tests pin every matrix entry with the basis vectors, and check
    skew-symmetry.

- **Smaller changes.**
  - pytest markers `gpu`, `budget` and `slow`; `check.sh` runs the CPU subset.
  - `check.sh` rejects unknown arguments, and takes its C++ file lists from git.
  - The `dev` extra is the single list of the toolchain's pip layer.
  - The setup script fails unless all three backends load, and derives compiler names from
    the host architecture.
  - The license is declared with an SPDX string.
  - Doxygen writes XML only.

- **Scaffold brought up to the standard.**
  - Numpy-style docstrings, Doxygen and rustdoc throughout.
  - Docstring examples run as tests.
  - Python tests: 14 -> 87.

## [0.1.1.0] - 2026-10-02

### Added

- **Design doc for the SLAM ladder** (`docs/design/2026-10-02-slamlab-ladder.md`). It covers the
  rungs from first-principles geometry to MegaSaM-style dense SLAM, including visual-inertial;
  each language's role; nine milestones, two of them reifyscene payoffs; the one-way boundary
  with reifyscene; edge results from budget-matched simulation; datasets and metrics; and
  renaming before the repository goes public.

- **Three-language scaffold.** One placeholder function, `hat` (the skew-symmetric matrix of a
  3-vector), is implemented in Python (`slamlab.so3`), C++ (`slamlab_core`, bound with nanobind
  as `slamlab._cpp`) and Rust (`slamlab-core`, bound with PyO3 as `slamlab_rs`).
  `slamlab.backends` picks an implementation at runtime. A parity test checks every built backend
  against shared fixtures in `tests/fixtures/`. The C++ and Rust cores also build and test
  standalone, without Python.

- **Conda environment, matching reifyscene.** `scripts/setup_slamlab.sh` builds the env with
  Python 3.11, cmake, ninja, a C++ compiler and Rust, all from conda-forge, then installs the pip
  layer from `scripts/locks/slamlab.lock` (`scripts/gen_env_lock.sh`). Each env gets its own C++
  build folder, `build/<env>`; two Python 3.11 envs sharing one CMake tree silently reused each
  other's cached compiler.

- **CI** (`.github/workflows/test.yml`): runs the Python suite with both extensions built, the
  standalone C++ tests (ctest), and Rust fmt, clippy and tests.

## [0.1.0.0] - 2026-10-02

### Added

- **Repository bootstrap.** `main`/`develop` branches, the `VERSION` file as the single
  version source, this changelog, the PolyForm Noncommercial 1.0.0 license, and the two tag workflows
  (`v<version>` on develop version bumps, `release-v<version>` plus a GitHub Release on
  develop -> main merges).
