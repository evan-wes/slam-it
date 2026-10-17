# Online SLAM (milestone 2B, PRs B1 to B4)

Milestone 2B, the second of the two milestones the ladder places before milestone 3
([Milestones 2A and 2B](2026-10-02-slam-it-ladder.md#milestones)): ORB-SLAM's own parts, so that
the edge rung is a SLAM system rather than odometry that starts again. Four PRs, each in Python,
C++ and Rust, each measured on the test set:
- **B1:** place recognition and relocalisation: a vocabulary of binary words, a keyframe
  database, and a lost track relocalised in its own map before a new one is started.
- **B2:** the covisibility graph and the essential graph, in place of the window of 10
  keyframes.
- **B3:** loop closure.
- **B4:** maps joined where a recognised place links them (ORB-SLAM3's merging).

Decided with evan-wes on 2026-10-14:
- **Order:** B1 first. It acts only once a frame has failed, so a run that never loses the track
  decides as before, and it is the vocabulary B3 and B4 build on.
- **The vocabulary,** DBoW2's (k = 10), in two sizes: 5 levels, about 100,000 words and 5 MB,
  committed and the default everywhere; 6 levels, ORB-SLAM's, about a million words and 50 MB,
  kept outside the repository and rebuilt exactly by the training tool, its SHA-256 in the
  repository. A run records which vocabulary it used.
- **Two training sets:** Bovisa 2008-09-01's front camera, ORB-SLAM's source, for a vocabulary
  comparable to ORB-SLAM's (it cannot be the same: its 10,000 images were never named, and our
  descriptors are not bit for bit ORB-SLAM's); and ARKitScenes' training fold, a phone's view of
  homes and offices, in visits the test set's captures are not from. B1 measures which serves the
  test set better.
- **A lost track** is relocalised against its own map for up to 3 s, as visual-only ORB-SLAM3
  does, and only then is the map closed and a new one started; a map of 10 keyframes or fewer,
  reset when lost, keeps A2's 5 failed frames.
- **The rest** (section 6): 7-Scenes' four smallest scenes; the two-view sample limit and the
  pipeline's state, deferred by A2's panel, in B1; the atlas in B4.

## 1. The problem, as measured

A2 lets a run go on after a lost track, and the test set showed what that leaves
(`docs/notes/test-set.md`):
- **348 losses over 61 runs** at seed 0, each starting a new map: a kept map spans a median of
  4.5 to 24.6 s by dataset and mode, and an ADVIO run's largest map holds a median third of the
  frames it tracks. Each map has its own frame and, monocular, its own scale.
- **ZJU's black frames** (B5, B6, B7): each black stretch of 1 to 3 s starts a new map, where
  the camera sees again what it saw before.
- **Why the track is lost:** 53% of the losses are frames placed by the motion model or PnP whose
  local map kept too few inliers, after a frame that kept a median of 35; 47% are frames no
  method placed.
- **Monocular runs lost deep into a map** (the ladder's open item L15): started a few frames
  later, 6 of 20 monocular runs on TUM and EuRoC lose the track 44 to 128 keyframes in, where
  ORB-SLAM relocalises.

Relocalisation (B1) answers the losses a recognised place can recover; the covisibility graph's
wider local map (B2) answers the ones where the window thinned.

## 2. How ORB-SLAM does it

From ORB-SLAM2 (Mur-Artal and Tardós, 2017) and ORB-SLAM3 (Campos et al., 2021), and their
source (`KeyFrameDatabase.cc`, `Tracking.cc`):
- **The vocabulary** (DBoW2; Gálvez-López and Tardós, 2012): a tree of binary words, built by
  hierarchical k-means (k-means++ seeding; a cluster's centre the bitwise majority of its
  descriptors) on ORB descriptors from images no evaluation uses (Bovisa), k = 10 and 6 levels;
  each word weighted by its inverse document frequency, log(N / N_i); two images compared by the
  L1 score of their normalised word vectors. Each keyframe also keeps its features grouped by the
  node 4 levels above the leaves (the direct index), so matching compares only features under
  the same node.
- **The keyframe database:** an inverted index, each word to the keyframes that contain it.
- **Relocalisation** (`Tracking::Relocalization`), against the active map:
  - candidates (`DetectRelocalizationCandidates`): the keyframes that share a word with the
    frame; those sharing more than 80% of the most words shared are scored; each score is
    accumulated over the keyframe's 10 best covisible keyframes that share a word with the
    frame, the best of them standing for the group; the groups above 75% of the best
    accumulated score are kept;
  - for each candidate, features matched word by word (`SearchByBoW`, a ratio of 0.75, with the
    rotation check); fewer than 15 matches discard it;
  - PnP RANSAC (MLPnP: confidence 0.99, at least 10 inliers, an expected inlier ratio of 0.5,
    at most 300 iterations, a minimal set of 6, the chi-square threshold 5.991), run 5
    iterations at a time across the candidates;
  - the pose optimised; fewer than 10 inliers discard it; under 50, more matches by projection
    from the candidate's map points (a window of 10 pixels, a distance of 100), optimised
    again, and at 30 to 50 a narrower search (3 pixels, 64); 50 inliers relocalise the frame;
  - the next frames need 50 local-map inliers, not 30, within `mMaxFrames` of it (the lite
    pipeline's `recovery_tracked` and `max_frames` already).
- **The loss flow (ORB-SLAM3, visual-only):** a frame whose frame-to-frame tracking fails puts a
  map of more than 10 keyframes into RECENTLY_LOST, a smaller one straight into LOST; a frame
  whose local map fails puts any map into RECENTLY_LOST. A RECENTLY_LOST frame is relocalised;
  after 3 s without success the state is LOST, and a LOST map is reset if it has 10 keyframes or
  fewer, kept in the atlas and a new one started otherwise.
- **The covisibility graph** (B2): keyframes linked when they share at least 15 points, weighted
  by how many; the local map is the keyframes that see the frame's points and their best
  neighbours. **The essential graph** keeps the spanning tree, edges of 100 shared points or more,
  and the loop edges.
- **Loop closure** (B3): candidates from the database, held by consistency across consecutive
  keyframes; the loop's Sim(3) by RANSAC over Horn's closed form; the points fused; the essential
  graph optimised in Sim(3); a global BA after. **Merging** (B4): the same recognition between
  the active map and the atlas's others, then the maps welded.

## 3. B1: place recognition and relocalisation (0.3.9.0 -> 0.3.10.0)

1. **The vocabulary, in rung 0** (`slam_it.foundations.vocabulary`, with C++ and Rust twins):
   - its files, a folder that eval reads (`slam_it.eval.vocabularies`, with readers in
     `slam_it_eval` and `slam-it-eval`; the rungs take a vocabulary, never its files), format
     version 2: `.npy` arrays of each node's descriptor (`(N, 32)` uint8), its parent, the first
     of its children and their count (children are consecutive), each leaf's word and each
     word's weight (float64); a header array (the format's and the descriptor's versions, k,
     levels and the words) and each array's SHA-256 (`sha256.npy`); and `training.json`, what
     training recorded (the training set, its images, the seed);
   - the transform: a descriptor descends from the root, at each level to the nearest child by
     Hamming distance (the lowest index among equals), to its word; a frame's words form its
     vector, weighted and L1-normalised; its features are grouped by their node at level 2
     (the direct index; ORB-SLAM's level 2 of 6, kept at level 2 of 5);
   - the score: DBoW2's L1 score, 1 - |v - w|_1 / 2;
   - the default is the committed 5-level vocabulary; a run's `--vocabulary` (`vo.run`'s
     `vocabulary`) names another folder, whose arrays must match their recorded checksums
     (Python checks them before any program runs; a native program run on its own trusts
     them, the ladder's one exception), and the run records the vocabulary it used
     (`run.json`'s `vocabulary`: its header and checksums, as every language writes them);
   - a vocabulary's words mean something only for the descriptor they were trained on: its
     header records the descriptor's version (`orb.DESCRIPTOR_VERSION`, which stands for the
     patch, the smoothing and the test pattern, pinned by their SHA-256), and every language's
     reader refuses another, rather than scoring against words that no longer mean anything.
     The feature count and the pyramid do not change a descriptor, so they are not checked.
2. **The training tool** (`scripts/train_vocabulary.py`, Python only): ORB features as the
   pipeline extracts them, from a training set's images, hierarchical k-means with k-means++
   seeding from a fixed seed and the bitwise majority as a centre, weights from the training
   images' document frequencies. Deterministic for a seed: a rebuild gives the same tree, its
   four integer and byte arrays to their checksums; its weights, numpy's log, may differ in the
   last bit on another CPU, so they are held to their rule.
   Three vocabularies, each from 10,000 images drawn with a fixed seed:

   | Vocabulary | Trained on | Levels | Kept |
   |---|---|---|---|
   | ORB-SLAM's kind | Bovisa 2008-09-01, the front camera | 6 | `../vocabularies/` |
   | A phone's | ARKitScenes' training fold | 6 | the same |
   | The default | ARKitScenes' training fold | 5 | committed, `slam_it/eval/vocabularies/default/` |

   The two kept outside are named, with their checksums and the commands that rebuild them, in
   `slam_it/eval/vocabularies/local.json`. If Bovisa's serves the test set better, the
   default is retrained from it.
3. **The keyframe database:** each keyframe's word vector and direct index made when it is made,
   and an inverted index from each word to the active map's keyframes, a keyframe removed as
   culling or a reset removes it. Ties are broken by keyframe id throughout, so that the three
   languages rank candidates alike.
4. **Relocalisation**, ORB-SLAM's `Tracking::Relocalization` with its thresholds (section 2),
   on rung 0's PnP RANSAC (`foundations.pnp`) and the edge rung's pose optimisation and
   projection search. Until B2's covisibility graph, a keyframe's best covisible keyframes are
   counted from the map's observations (`kp_point`): those that share the most points with it,
   at least 15. A neighbour that shares words but was not scored adds nothing to the group
   (ORB-SLAM adds the score it kept from an earlier query, which the three languages could not
   reproduce alike).
5. **The loss flow:** a frame that fails, as now (the motion model, then PnP against the local
   map), is relocalised against the whole active map, while `relocalize` is on (by default, as
   ORB-SLAM always relocalises; on, a run needs a vocabulary: `vo.run` takes the committed one
   unless told another, and a program needs `--vocabulary`; off, it refuses one, and a map is
   lost as A2 loses it). A run is recently lost from its first failed frame: it is tracked
   again on a frame that succeeds,
   and lost, its map closed as A2 closes it, once `relocalize_seconds` (3.0) have passed since
   that frame with no success. A map of `reset_keyframes` or fewer is lost after `lost_frames`
   (5) failures instead, as section 6 has it. The pipeline's own state stays tracking or not
   initialised between frames, the loss flagged on the frame's step (the A2 panel's A8).
6. **The run format,** version 5, a clean break with 0.3.9.0's runs (B1's panel, H9): `run.json`'s
   `vocabulary` is required, null or the vocabulary's record; a frame relocalised is placed by
   `relocalization`, and
   its diagnostics say how relocalisation went (the candidates, each one's matches and inliers,
   the outcome), null where it did not run.
7. **The measure, the bake and the status page:** relocalisations counted per run beside the
   losses; the viewer's timeline marks a relocalised frame; the status page says how many
   frames tracking failed and relocalisation found again. A count of frames, not of times the
   track was dropped, decided with evan-wes on 2026-10-15: relocalisation runs on the frame
   tracking failed, so a relocalised frame can follow a tracked one.
8. **Parity:** the golden record gains a room where the track is lost and found again: the lost
   room's blank frames end where the camera sees what it saw before, so the next frame
   relocalises in map 0 instead of starting map 1. Its rooms that never lose the track keep
   their records unchanged. From B1's panel: the window room (found again, then lost by
   `relocalize_seconds` before `lost_frames`, an RGB-D run culling keyframes first);
   relocalisation's settings each moved where it alone decides; a golden bundle of the found
   room for the viewer's tests; and native tests of the vocabulary, its reader, the database,
   relocalisation and the pipeline relocalising.
9. **Rust stays**, as it was always meant to: no language is dropped partway through the
   project, so no keep-or-drop decision follows this PR (B1's panel, A13).
10. **The two-view sample limit,** deferred from A2's measurement: a failed monocular start
   takes up to 0.43 s on ADVIO, every late frame of the test set, where rung 0's two-view RANSAC
   draws up to 10,000 samples per model (`twoview.Options.max_iterations`) and ORB-SLAM's
   `TwoViewReconstruction` 200. Its own commit, first, since it changes every monocular start,
   measured apart from relocalisation.

## 4. B2 to B4, in outline

Each gets its own section here before its PR, its forks put to the owner then.
- **B2, the covisibility graph:** section 8.
- **B3, loop closure:** inline, not in a thread, as the ladder decided; the frame it closes on
  measured against the deadline, a thread only if it misses it. Measured on TUM and EuRoC's
  loops.
  From B2's latency (section 9), decided with evan-wes on 2026-10-16: B3's design also decides
  how the edge rung stands in for ORB-SLAM's threads, local mapping's refusal of a keyframe
  while busy and its interrupted local BA with loop closing's own, measured together; B4's
  merging runs in loop closing's, as ORB-SLAM3's does.
- **B4, merging maps:** the atlas the A2 panel deferred (`edge.atlas`: a kept map's descriptors
  and observations kept whole, the pipeline owning the trajectory record), and maps welded where
  the database recognises one in another. With it, the loss window revisited: B1 relocalises a
  lost track for 3 s before a new map starts, as visual-only ORB-SLAM3 does, and on the test
  set the frames waited through cost more than relocalisation found (B1's note). Once a lost
  map can be found again by merging, a new map could start at once instead, nothing lost to
  the wait. B1 keeps 3 s (decided with evan-wes on 2026-10-15). TUM's `fr2_360_kidnap` and
  7-Scenes' relocalisation across sequences, deferred from B1, are measured here: each needs a
  map other than the active one recognised. From B1's panel (L7): a kept map needs its
  database, kept or rebuilt from its descriptors, and B4 decides whether words are counted
  across maps, as ORB-SLAM3's `DetectRelocalizationCandidates` counts them, or per map, as B1's
  database does. Decided with evan-wes on 2026-10-16: the record of a lost map's strong links,
  which B2 computes in the frame the map is lost on (58 to 360 ms, growing with the map; section
  9), stays there until the atlas keeps maps whole.

## 5. Measurements (B1)

- **The two-view sample limit first,** the monocular runs at seeds 0, 1 and 2 against A2's:
  the starts, the share tracked and the late frames.
- **The test set at seed 0**, against the same code with relocalisation off, with the default
  vocabulary: the share tracked, the maps per run, the losses and relocalisations, how each
  stretch of failed frames ended, and each relocalised frame's error against the ground truth
  (measured side by side by `scripts/measure_sharded.py`, compared by
  `scripts/compare_testsets.py`).
- **The latency alone**, each run pinned to one core: the five sequences that relocalise most,
  relocalisation on and off; a relocalised frame's latency, and a failed attempt's, against the
  frame's deadline.
- **ZJU's black frames:** whether each black stretch ends found again in its first map.
- **A relocalising run in the viewer:** ZJU B0's.
- **Cut** by evan-wes on 2026-10-15, to move on to B2: seeds 1 and 2, and the two 6-level
  vocabularies' comparison (to the backlog); TUM's `fr2_360_kidnap` and relocalisation across
  7-Scenes' sequences (to B4, which recognises a map other than the active one).

## 6. The training sets, and the rest decided

**The training sets** (both, decided with evan-wes on 2026-10-14):
- **Bovisa 2008-09-01** (Rawseeds, Politecnico di Milano): a robot's front camera, indoor and
  outdoor, static: 65,047 colour frames of 320x240 at 30 Hz over 36 minutes, the 65,047 images
  ORB-SLAM's 10,000 were drawn from. Its torrents' tracker is gone; the files are on AIRLab's
  Dropbox. The front camera's tar, 9,338,634,240 bytes, matches its torrent's SHA-1 hashes in
  all 8,907 pieces. Rawseeds states no licence: a research benchmark, used here for a local
  vocabulary in a private, non-commercial project. A Bovisa-trained default committed to the
  repository would be a derivative of it, to be weighed then.
- **ARKitScenes' training fold:** Apple's folds are split by visit (one scan of one place): of
  1,664 visits one has captures in both, and the test set's 10 captures come from 10 visits in
  the validation fold alone. The training captures are drawn from the training fold's visits,
  skipping the one in both folds and the 8 captures with no visit. About 20 captures, every
  fourth frame, give about 10,000 images, about 15 GB of 640x480 frames (checked with its
  download script first). CC BY-NC-SA 4.0.

**The rest** (decided with evan-wes on 2026-10-14):
- **7-Scenes:** its four smallest scenes, heads, stairs, fire and chess, 7.8 GB (all seven are
  21.6 GB); 640x480 RGB-D with ground truth, for non-commercial use.
- **The riders the A2 panel deferred:** the pipeline's own state two-valued, the loss flagged
  on the step (A8's rest), with B1's loss flow; the atlas (A4, A9, H7) with B4, where maps must
  be kept whole; the two-view sample limit (section 3, item 10) in B1, measured apart.
- **A small map** keeps `lost_frames` (5): ORB-SLAM3 loses a map of 10 keyframes or fewer at
  once when its frame-to-frame tracking fails, and resets it, as such a map would be after
  relocalising; a larger map relocalises for up to 3 s.

## 7. Plan

- **B1** (0.3.9.0 -> 0.3.10.0): the two-view sample limit, measured; the training sets and
  7-Scenes downloaded (sizes checked first); the vocabulary in three languages and the training
  tool; the three vocabularies trained, the default committed; the database and relocalisation;
  the loss flow; the diagnostics, measure, bake and status page; the relocalising room in the
  golden record; the test set rerun and the note. The review panel on the finished slice;
  offline CI before the merge.
- **B2** (0.3.10.0 -> 0.3.11.0, section 8): the settings grouped; the covisibility module and the
  spanning tree, held to a brute-force fixture; tracking's local map and the reference keyframe;
  local mapping off the window; the fallback and relocalisation's groups; the run format 6 and
  the bundle's 3; the viewer's graph; the golden record regenerated; the test set rerun and the
  note. The review panel on the finished slice; offline CI before the merge.
- **B3 and B4** after it, each designed here first.
- **Not in 2B:** the IMU (milestone 6); the learned rungs.

## 8. B2: the covisibility graph (0.3.10.0 -> 0.3.11.0)

Decided with evan-wes on 2026-10-16. The window of the last 10 keyframes goes: each of its six
users takes its keyframes from the covisibility graph, as ORB-SLAM's do (ORB-SLAM2's
`Tracking.cc`, `LocalMapping.cc`, `KeyFrame.cc` and `Optimizer.cc`; ORB-SLAM3's where it
differs).

1. **The settings grouped first** (B1's panel, A12), in a commit of their own: `edge.Options`'
   92 flat settings nested by the stage that reads them (features, matching, stereo,
   initialization, tracking, keyframes, mapping, adjustment, covisibility, relocalization and
   loss; `seed` at the top), each group's prefix dropped from its names (`reloc_min_inliers` is
   `relocalization.min_inliers`), in the three languages. A clean break: `--option` takes
   dotted keys (`--option mapping.fuse_window=3`), and the run format's `settings` are nested
   (item 7).
2. **The graph, counted when asked** (`slam_it.edge.covisibility`, with C++ and Rust twins, in
   place of `edge.database.best_covisible`): two keyframes' weight is the number of points both
   observe, counted from the map's observations each time it is asked for, so it is always
   exact. ORB-SLAM stores the weights and updates a keyframe's at a few points
   (`UpdateConnections`: at initialisation, when it is inserted, after fusion, and in loop
   closing), so they are stale in between: an observation BA drops still counts. A keyframe is
   linked to those it shares at least `covisibility.min_points` (15) with, or, if it shares that
   many with none, to the one it shares most with; its best are its links, the most shared
   first, ties broken by the lower id. Only the spanning tree is stored.
3. **The spanning tree,** the essential graph's spine, kept for B3: each keyframe's parent, a
   new map array (`kf_parent`), is the keyframe it is most strongly linked to when it is
   inserted with its tracked points (ORB-SLAM's first `UpdateConnections`). A keyframe that
   shares no point then gets its parent after fusion; a map's first keyframe has none. A
   keyframe's children are those whose parent it is, found when asked. A culled keyframe's
   children get new parents as ORB-SLAM's `SetBadFlag` gives them: the candidates start as its
   parent; each round, the child and candidate most strongly linked are paired and the child
   becomes a candidate; children linked to no candidate get its parent. The essential graph's
   strong links (`covisibility.strong_points`, 100) are counted the same way; its loop links are
   B3's.
4. **Tracking's local map** (`UpdateLocalKeyFrames`): K1, the keyframes that observe the points
   the frame was matched to, the most shared first; then, for each of K1 in that order, unless
   the local map already holds more than `covisibility.local_keyframes` (80): the first of its
   `covisibility.local_neighbours` (10) best not yet in it, its first child not yet in it (the
   lowest id), and its parent if not yet in it. Its points are those its keyframes observe. The
   keyframe of K1 that shares the most points with the frame is its **reference keyframe**,
   which the keyframe decision counts its tracked points against (`NeedNewKeyFrame`'s
   `nRefMatches`), in place of the newest keyframe. **The paper's expansion, not the
   source's:** ORB-SLAM2's and ORB-SLAM3's loops end the whole expansion once a parent is
   added (a `break` out of the outer loop), and take K1 in the order of its keyframes'
   addresses in memory. The papers' K2 is every K1 keyframe's neighbours, so that is what is
   done, in an order the three languages share.
5. **Local mapping:**
   - **local BA** (`LocalBundleAdjustment`): the new keyframe and every keyframe linked to it;
     the keyframes that observe their points, and the map's first keyframe, held fixed, as now;
   - **keyframe culling** (`KeyFrameCulling`): every keyframe linked to the new one, by the
     same test as now;
   - **triangulation** (`CreateNewMapPoints`): the new keyframe's best
     `mapping.triangulation_neighbours` (10; monocular, 20);
   - **fusion** (`SearchInNeighbors`): its best `mapping.fuse_neighbours` (10; monocular, 20)
     and each one's best `mapping.fuse_second_neighbours` (5), leaving out the new keyframe and
     repeats. ORB-SLAM2's numbers, as triangulation's already are; ORB-SLAM3's (30 monocular,
     20 second neighbours) are a setting away.
6. **The local fallback,** PnP against the local map's points as now, searches the local map
   built from the last tracked frame's points (item 4) in place of the window's. Relocalisation's
   candidate groups (B1) take each candidate's best `relocalization.neighbours` (10) from the
   graph, which links a keyframe sharing fewer than 15 points with every other to its best, as
   ORB-SLAM's `GetBestCovisibilityKeyFrames` does.
7. **The run format, version 6, and the bundle's, version 3,** clean breaks: `run.json`'s
   `settings` nested (item 1); each keyframe's parent, the last it had
   (`keyframe_parents.npy`, `(K,)` int64, -1 for none); and each map's strong links when it
   closed or the run ended (`keyframe_links.npy`, `(L, 3)` int64: the two keyframes' ids and the
   points they share). The diagnostics count the local map's keyframes and points, name the
   reference keyframe, and count local BA's free and fixed keyframes.
8. **The viewer** draws the graph on a toggle, as ORB-SLAM's viewer does: each living
   keyframe's line to its parent, and the strong links between living keyframes.
9. **Parity:** B2 changes every run's decisions, so the golden record and the golden bundle are
   regenerated whole, their rooms unchanged. The covisibility module is tested in the three
   languages against a brute-force fixture (the weights, the links, the best, the local map,
   and a culled keyframe's children given new parents), and each stage moved off the window
   keeps its native tests.

## 9. Measurements (B2)

Once, on the finished branch:
- **The test set at seed 0,** against B1's (0.3.10.0), side by side (`scripts/measure_sharded.py`,
  compared by `scripts/compare_testsets.py`): the share tracked, the losses and what failed at
  each, the maps, the relocalisations and the ATE, with the local map's and local BA's sizes
  from the diagnostics. In particular: the losses whose local map kept too few inliers (53% of
  A2's, section 1), and the monocular runs lost deep into a map (the ladder's L15).
- **The latency alone,** each run pinned to one core, on the five sequences whose maps grow
  largest: keyframe frames apart from the rest, against the deadline. Local mapping runs
  inline, and local BA now covers every linked keyframe; a late frame is put to evan-wes with
  the numbers, not met by a cap decided alone.

*As measured* ([the test-set note](../notes/test-set.md#b2-the-covisibility-graph)): 208,906 of
the set's 246,529 frames tracked against B1's 200,897, in 226 maps against 268, with 177 losses
against 231; the losses whose local map kept too few inliers 91 against 152; none of L15's 20
runs lost. Tracking keeps its deadline but for 172 of the 35,066 frames made alone, and
mapping, judged as a thread of its own, is loaded 1.7 to 2.6 times over: never busy, the
pipeline refuses no keyframe and cuts no local BA short. Put to evan-wes, who left the threads
to B3's design (section 4).
