# Online SLAM (milestone 2B, PRs B1 to B4)

Milestone 2B, the second of the two milestones the ladder places before milestone 3
([Milestones 2A and 2B](2026-10-02-slam-it-ladder.md#milestones)): ORB-SLAM's own parts, so that
the edge rung is a SLAM system rather than odometry that starts again. Five PRs, each in Python,
C++ and Rust, each measured on the test set:
- **B1:** place recognition and relocalisation: a vocabulary of binary words, a keyframe
  database, and a lost track relocalised in its own map before a new one is started.
- **B2:** the covisibility graph and the essential graph, in place of the window of 10
  keyframes.
- **B3:** loop closure, in the one worker that tracks and maps.
- **B3b:** tracking, mapping and loop closing in threads of their own.
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
- **B3, loop closure:** section 10.
- **B3b, the threads** (section 12; decided with evan-wes on 2026-10-17, replacing the stand-in
  for ORB-SLAM's threads decided on 2026-10-16): real threads in the three languages, a design
  of our own started from ORB-SLAM's, which runs tracking, local mapping, loop closing and
  global BA each in a thread, with queues between them and its handshakes (`AcceptKeyFrames`,
  `InterruptBA`, `RequestStop`). Ours is compared with ORB-SLAM's own scheme, built in our C++
  so that only the threading differs; ORB-SLAM3 itself, built and run here, is compared with the
  whole system once phase B is done (decided with evan-wes on 2026-10-17). A switch runs the
  threads' work in order, for debugging and each language's record. Python runs the same design
  under the GIL: it stays on 3.11, its free-threaded build passed over, as GTSAM has none. The
  latency bar, C++ and Rust's alone, starts here. B4's merging runs in loop closing's thread, as
  ORB-SLAM3's does. From B2's review panel, for this design:
  - ORB-SLAM3's culling tests every keyframe sharing a point with the new one, stopping after
    100 (L4); B2's tests the new one's links;
  - the papers' K2, which B2 builds, makes a larger local map than ORB-SLAM's source does (its
    `break`), so ORB-SLAM's timings are not B2's to compare with (L6).
- **B4, merging maps** (section 14): the atlas the A2 panel deferred (`edge.atlas`: a kept map's
  descriptors and observations kept whole, the pipeline owning the trajectory record), and maps
  welded where the database recognises one in another. With it, the loss window revisited: B1
  relocalises a lost track for 3 s before a new map starts, as visual-only ORB-SLAM3 does, and
  on the test set the frames waited through cost more than relocalisation found (B1's note).
  Once a lost map can be found again by merging, a new map could start at once instead, nothing
  lost to the wait. B1 keeps 3 s (decided with evan-wes on 2026-10-15). TUM's `fr2_360_kidnap`
  and 7-Scenes' relocalisation across sequences, deferred from B1, are measured here: each needs
  a map other than the active one recognised. From B1's panel (L7): a kept map needs its
  database, kept or rebuilt from its descriptors, and B4 decides whether words are counted
  across maps, as ORB-SLAM3's `DetectRelocalizationCandidates` counts them, or per map, as B1's
  database does. Decided with evan-wes on 2026-10-16: the record of a lost map's strong links,
  which B2 computes in the frame the map is lost on (58 to 360 ms, growing with the map; section
  9), stays there until the atlas keeps maps whole. From B2's review panel (run 2, A15): the
  map's erase guards parents, not the root, so merging that renumbers a root revisits the rule
  kept in `covisibility.cull`.

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
  download script first). Apple's ARKitScenes licence.

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
- **B3** (0.3.11.0 -> 0.3.12.0, section 10): GTSAM in the three languages; detection, the loop's
  Sim(3), the correction, the essential graph and global BA; the final trajectory; the run
  format 7 and the bundle's 4; a record per language, the loop room and the bar; the loop
  sequences and the test set measured. The review panel on the finished slice; offline CI
  before the merge.
- **Global BA's damping** (after B3, its own PR; B3's review panel, L2, section 11): scaled
  Levenberg-Marquardt damping in the three languages, measured on the loop sequences and the
  test set, the records regenerated.
- **B3b** (0.3.13.0 -> 0.3.14.0, section 12): the seams, which change no decision; the threads,
  our design and ORB-SLAM's scheme in our C++; global BA's thread; the run format 8; the tests
  and ThreadSanitizer; the latency bar, and the loop sequences and the test set threaded. The
  review panel on the finished slice; offline CI before the merge.
- **B4** (0.3.13.0 -> 0.3.14.0, section 14): the atlas; relocalisation in every map; merge
  detection and the merge; sessions; the run format 9 and the bundle's 5; the merge room and the
  records; multi-session EuRoC, the kidnap, 7-Scenes, the loss window, and the test set. The
  review panel on the finished slice; offline CI before the merge.
- **Built side by side** (decided with evan-wes on 2026-10-17): B3b and B4 in worktrees of their
  own, B4 on B3b's seams. They merge in order, B3, then B3b, then B4, each PR opened once the one
  before it has merged.
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
   B3's. *As built (B2's review panel, A1 and A3):* a keyframe leaves the map only by
   `covisibility.cull`, the children reparented and then the keyframe erased, which refuses the
   map's first keyframe, the tree's root, as `SetBadFlag` leaves it; `map.erase_keyframe`
   refuses a keyframe that is still a parent; `adopt` is for the keyframe being inserted, as on
   an older one it can close a cycle.
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
   keeps its native tests. *As built (B2's review panel, L1):* the fixture also holds `cull` and
   what the stages take from the graph, fusion's targets and local BA's keyframes
   (`mapping.fuse_targets`, `mapping.local_ba_keyframes`, public for it), each of fusion's counts
   binding; every map of the golden record is one tree; and B2's settings are moved off their
   defaults in the whole runs the languages are compared on.

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

## 10. B3: loop closure (0.3.11.0 -> 0.3.12.0)

Decided with evan-wes on 2026-10-17, following ORB-SLAM's papers and a reading of its source
(ORB-SLAM2's and ORB-SLAM3's `LoopClosing.cc`, `KeyFrameDatabase.cc`, `Sim3Solver.cc` and
`Optimizer.cc`). Loop closing runs on each keyframe after local mapping, in the one worker, as
everything has until now; B3b gives it a thread. Three decisions of that day reach beyond B3:
- **Well-tested libraries are welcome in the runtime,** the SLAM parts included: learning SLAM
  includes learning its standard tools. Here that is GTSAM, in all three languages.
- **Each language is held to a bar of results on the same data,** not to the others'
  decisions: each must map and localise well, and none need decide as another does.
- **The trajectory a run scores is ORB-SLAM's final one,** which loop closure corrects.

1. **GTSAM 4.3 is the back end** (Dellaert and Kaess's factor-graph library), behind rung 0's
   `slam_it.foundations.graphs`: the loop's Sim(3) refinement, the essential graph and global
   BA.
   - C++ links conda-forge's build of it, made against our Eigen 5.0.1. Python uses its
     official bindings, the same build. Rust reaches it through a `cxx` bridge. *As built
     (decided with evan-wes on 2026-10-17):* `setup_slam_it.sh` builds GTSAM 4.3.0 itself, and
     the worker's image its C++ library the same way, for the C++ program it runs (its Python,
     which imports GTSAM but never solves with it, takes PyPI's 4.3.0 wheel, pinned by its hash:
     B3's review panel, A3): conda-forge's build blocks Eigen's products by the caches
     of the core that runs first, and its global BA came out different in the last bits on a
     P-core and an E-core of this machine, where our cores compile with `EIGEN_NO_CPUID` so a run
     repeats on any core. Ours compiles with it too; static (METIS and its bundled CCOLAMD too,
     no CHOLMOD), so nothing needs a library path at run time; single-threaded (no TBB), like
     the cores; its Python wrapper built over the same library, its largest file needing over
     8 GB to compile, so that step runs one file at a time under a 16 GB cap. The bridge's one
     module is the only place `slam-it-foundations` allows `unsafe`, which `cxx`'s
     `unsafe extern "C++"` needs.
   - GTSAM lacks two of the factors needed: a point's reprojection through our camera models
     (radtan with its `k3`, which its `Cal3DS2` leaves out, and the right column), and a match's
     reprojection through a Sim(3). Each language writes them on its own camera model: C++ as
     GTSAM factors; Python as `CustomFactor`s, its reference; Rust through a generic factor in
     the bridge that calls back into Rust for the residual and its Jacobians, as GTSAM's
     `CustomFactor` calls back into Python. *Revised with evan-wes on 2026-10-17,* while building:
     the design first had them written once in C++ and compiled into the Rust crate, which would
     make Rust's build a second, hand-kept build of part of the C++ core, and leave Rust none of
     the maths. *As built (decided with evan-wes on 2026-10-17):* Python's bundle adjustment
     takes GTSAM's own factors through a pinhole camera (`GenericProjectionFactorCal3_S2`, and
     `GenericStereoFactor3D` with a right column), which model it exactly: GTSAM's Python wrapper
     copies a problem's values on every call into a `CustomFactor`, and global BA on the loop
     room's map took 274 s through them, 0.73 s through GTSAM's. *As built:* every language's
     BA factor gives a point behind its camera GTSAM's own factors' constant error, twice the
     focal length a row, with no gradient; with none, global BA (no robust loss) collapsed a
     monocular map, its points moved behind their cameras until nothing was left to cost (the
     loop room in C++: final error 0.0, final ATE 50 cm against 15 cm as tracked).
   - A monocular map optimises Sim(3) (`Similarity3`); a stereo or RGB-D map fixes the scale
     and optimises SE(3) (`Pose3`), as ORB-SLAM's `mbFixScale` does.
   - GTSAM joins the runtime dependencies. The import-linter contract that kept the test oracles
     out of the library drops scipy, and a test that used one as an oracle keeps it wherever the
     library does not call the same routine.
2. **Detection, ORB-SLAM3's** (`NewDetectCommonRegions`), on the active map's keyframe database.
   A map of fewer than `loop.min_keyframes` (12) keyframes is not searched. *As built:* the
   settings' group is `loops` (`loops.min_keyframes`, and so on), `loop` being a keyword in
   Rust. Where ORB-SLAM3's source does not do what its design means (the matches by word
   merged last-wins, `Sim3Solver`'s inverted test of which keyframe a match is in,
   `OptimizeSim3` passed the candidate rather than the keyframe that matched most, a later
   keyframe's last search through the unrefined similarity with its scale set to 1, the
   group's widening past the new keyframe's links), the build does what the design means:
   `slam_it.edge.loops`'s docstring lists each.
   - **Candidates** (`DetectNBestCandidates`): the keyframes sharing a word with the new one and
     not linked to it; those sharing more than 80% of the most words any shares are scored; each
     score accumulated over the keyframe's 10 best neighbours that were scored, the best-scored
     standing for the group; the `loop.candidates` (3) best groups, best first, ties by id.
   - **Each candidate,** with its 10 best neighbours, unless one of them is linked to the new
     keyframe: features matched by word against each (a ratio of 0.9), at least 20 in all; the
     loop's Sim(3) (item 3); the group's points projected through it (8 pixels, a ratio of 1.5),
     at least 50; refined, at least 20 inliers; projected again (5 pixels, 1.0), at least 80.
   - **Confirmed** by up to 3 of the new keyframe's 10 best neighbours, each projecting at least
     30 of the group's points through the Sim(3) carried to it. Three confirmations close the
     loop at once. Fewer are carried to the keyframes that follow, each confirming with 30
     points projected, 50 inliers refined and 100 projected again, until there are three (the
     loop closes) or two misses in a row (it is dropped).
3. **The loop's Sim(3):** RANSAC over samples of 3 matches (rung 0's `ransac`: a probability of
   0.99, at least 15 inliers, at most 300 iterations; each candidate run to its end, best first,
   as B1's relocalisation runs them). Each sample is solved in closed form by rung 0's
   `alignment.umeyama`, the least-squares answer Horn's quaternion method gives ORB-SLAM's
   `Sim3Solver`, with Umeyama's guard against a reflection; the scale is fixed for stereo and
   RGB-D. A match is an inlier when it reprojects within the 99% chi-square bound (9.21 σ²) in
   both keyframes. The result is refined as `OptimizeSim3` refines it, on GTSAM: each match
   reprojected both ways, a Huber loss at √10, 5 iterations, the matches past 10 dropped, then
   10 more (5 if none was dropped), with no robust loss, as `OptimizeSim3`'s second round
   removes its kernels.
4. **The correction, ORB-SLAM's `CorrectLoop`:**
   - the new keyframe and its links moved by the loop's Sim(3), their points with them;
   - each matched point replacing the new keyframe's (`Replace`), or joining it where it had
     none;
   - the loop's group's points fused into the new keyframe and its links (a 4-pixel window, a
     ratio of 0.8); *as built,* with no ratio: ORB-SLAM's loop `Fuse` is passed the 0.8 but
     never reads it, the nearest keypoint within `TH_LOW` taking the point;
   - the links new across the loop collected, then the essential graph (item 5);
   - the loop edge added to both keyframes.
5. **The essential graph,** ORB-SLAM's `OptimizeEssentialGraph`, on GTSAM:
   - a node per keyframe, starting from its corrected pose where step 4 gave it one;
   - an edge between two keyframes for each of: the spanning tree, the loop edges, links
     sharing at least `loop.essential_points` (100) points, and the loop's new links of that
     strength, the new keyframe's link to the matched one whatever its strength. Each edge holds
     the two keyframes' relative pose from before the correction; *as built,* but the loop's
     new links, which hold theirs between the corrected poses, as ORB-SLAM's code measures them;
   - unit information, the matched keyframe fixed, Levenberg–Marquardt for 20 iterations (as
     ORB-SLAM2 fixes it; ORB-SLAM3 fixes the map's first keyframe);
   - each point then moved with its reference keyframe, or with the keyframe that moved it in
     step 4.
6. **Global BA after each loop,** as visual-only ORB-SLAM3 always runs it, on GTSAM:
   - every keyframe and point of the map, the reprojection factors with no robust loss (ORB-SLAM's
     loop closing calls `GlobalBundleAdjustemnt` with `bRobust` false; the design first gave it
     a Huber loss in error), the map's first keyframe fixed, 10 iterations;
   - each keyframe and point set to its result. It runs inline here, so no keyframe is made
     while it runs and none needs correcting through the spanning tree afterwards, as ORB-SLAM's
     `RunGlobalBundleAdjustment` corrects them; B3b's thread does need that;
   - `loop.global_ba` turns it off, so that what it adds can be measured.
7. **What stays guarded** (B2's review panel):
   - culling spares a keyframe with a loop edge, and the keyframes detection is holding, as
     ORB-SLAM's `SetNotErase` does, in `covisibility.cull` (L9);
   - a local BA with no keyframe held fixed is skipped, as ORB-SLAM3 skips it (L5).
8. **The final trajectory** (ORB-SLAM's `SaveTrajectoryTUM`):
   - as each frame is tracked, it keeps its reference keyframe and its pose relative to it
     (`mlRelativeFramePoses`); a culled keyframe keeps its pose relative to its parent
     (`mTcp`);
   - when a map closes or the run ends, each of its frames is placed by its reference keyframe's
     final pose, through the keyframe's culled ancestors where it was culled;
   - the poses as tracked are kept beside it;
   - *as built,* after a correction the last frame, the new keyframe's own, moves with it, as
     ORB-SLAM's `UpdateLastFrame` places the last frame by its reference keyframe.
9. **The run format, version 7, and the bundle's, version 4,** clean breaks:
   - `trajectory.txt` holds the final trajectory, and `trajectory_tracked.txt` the poses as
     tracked;
   - each keyframe's loop edges are kept (`keyframe_loops.npy`, `(E, 2)` int64);
   - per keyframe, the diagnostics record how detection went (the candidates, their matches and
     inliers, the confirmations, the outcome), and, on a loop, its correction (the matched
     keyframe, the Sim(3), the points fused, the graph's edges, iterations and cost, and global
     BA's), with their times; *as built,* the time is the frame's `loop_seconds`, one of the
     run's steps, not part of the stage's summary, which runs compare and the records hold;
   - the viewer draws the loop edges.
10. **Tests: a record per language, and a bar for all three:**
    - the golden record becomes one per language (`room_runs_{python,cpp,rust}.json`), each its
      own language's regression check, regenerated deliberately as now;
    - the stage comparisons with Python's reference stay, at their tolerances;
    - a new room whose camera comes back to where it started, so a loop closes, in every mode;
      *as built,* a circle of 0.6 m looking outward and 0.25 rad up, turning 420° in 280
      frames (`tests/edge/room.py`'s `loop_path`): looking level at a wall 2.4 m off, RGB-D's
      tracking jumped by up to 3°, a turn and a step sideways alike at one depth;
    - the bar on the rooms, in layer 1, for all three languages: the frames tracked, the maps,
      the loops closed, no loop at a wrong place, and the error of the final trajectory against
      the room's truth and of the points against its walls, each threshold set from B3's first
      runs; *as built* (`tests/orchestration/test_loop_room.py`), about half again those runs'
      (the same in the three languages but monocular's floats): every mode closes a loop, in one
      map, 276 of 280 frames tracked in monocular and all otherwise (at least 95% needed); the
      final ATE 4.67 cm in RGB-D, 3.64 cm in stereo and 1.14 to 1.34 cm in monocular (7, 5.5
      and 2 cm the bounds), each below the poses' as tracked; the points' median distance to
      the walls 7.3, 7.1 and 3.9 to 4.3 cm (11, 11 and 6.5 cm the bounds); the loops' keyframes
      0.14 to 0.34 m apart and turned 12° to 33° by the truth;
    - *as built,* the whole runs are no longer compared across languages; their records, made
      from the same code on the same rooms, decide alike in all 27 cases today, and a native's
      reading of its settings (H21) is held in its own language, its decisions changing with
      them;
    - the bar on real data, C++ and Rust's, is section 11's;
    - a loop is judged against the ground truth twice (decided with evan-wes on 2026-10-17):
      - **a wrong place,** which fails the bar: by the ground truth, the loop's two keyframes
        were more than 3 m apart or looked more than 60° apart, so they could not have seen the
        same place (place recognition's convention; starting values, revisited with the first
        runs); *as built,* the angle between their viewing directions: the whole turn between
        them first counted a roll about the view, and flagged a loop on TUM's
        fr2_360_hemisphere whose keyframes turned 88° but looked 18° apart, its corrected
        relative pose within 0.09 m and 2.3° of the truth;
      - **a poor pose,** which is reported: with the map aligned to the ground truth as the ATE
        aligns it, the two keyframes' relative pose misses the truth by more than five times the
        map's median error, or by more than 5°; *as built,* their final relative pose, at the
        run's end, after every later local BA and loop, not the poses just after the correction
        (B3's review panel, L7), judged by `slam_it.eval.loops`, for the rooms and the measure
        alike (A7).

## 11. Measurements (B3)

Once, on the finished branch, as verification goes from B3 on (decided with evan-wes on
2026-10-17): C++ and Rust measure what the PR claims; Python is held to the bar on the rooms
alone, and runs on real data only when asked.
- **The bar's anchor is ORB-SLAM's published accuracy** (decided with evan-wes on 2026-10-17):
  the RMSE of the ATE that ORB-SLAM3's paper gives for EuRoC, monocular and stereo, ORB-SLAM2's
  for TUM RGB-D, and ORB-SLAM's for TUM monocular, each checked against its paper. Our headline
  ATE is the same measure. A sequence and mode with no published figure has no ATE bar until
  ORB-SLAM3 is built and run here, after phase B. *The protocols differ* (B3's review panel,
  L6): ORB-SLAM3's EuRoC figures are the median of 10 runs, its monocular ones with its atlas
  merging maps (V1_03 monocular, its paper says, is solved by merging); the bar compares one run
  at seed 0, without merging, which arrives in B4. Some of the runs that track under 95% of
  their frames may be what B4's merging addresses.
- **The loop sequences**, C++ and Rust at seed 0, sharded:
  - TUM's fr3_long_office_household, fr1_desk and fr3_nostructure_texture_near_withloop,
    monocular and RGB-D;
  - EuRoC's MH_01 to MH_05 and V1_01 to V1_03, monocular and stereo;
  - each run's loops closed, any at a wrong place or with a poor pose, the share tracked, the
    maps and the final trajectory's ATE, against the published figure, with global BA on and
    off.
- **The bar's starting thresholds,** on each loop sequence: the ATE within 1.5 times the
  published figure, at least 95% of the frames tracked (the papers mark a sequence only as
  tracked or failed, so there is no published share to anchor it), and no loop at a wrong
  place.
- **The test set at seed 0, C++ and Rust** (decided with evan-wes on 2026-10-17), against
  B2's: the loops closed, any at a wrong place or with a poor pose, the share tracked, the maps,
  and the ATE of both trajectories (the as-tracked one compares with B2's). B2's runs are run
  format 6, so their per-batch summaries are written before the break (A9).
- **Not measured:** latency. Loop correction and global BA run inside their frame here, so the
  frames they make late are counted from the diagnostics, and judged from B3b on.
- **As measured** (2026-10-17; [the test-set note](../notes/test-set.md#b3-loop-closure)):
  - on the loop sequences, 16 of the 22 runs meet the bar, in C++ and Rust alike and with
    global BA on or off. fr3_long_office RGB-D misses its ATE bound (1.9 cm against 1.5; 5.9 cm
    as tracked); five track under 95% of their frames: fr1_desk monocular (89.9%), MH_05
    monocular (94.6%), V1_02 monocular (94.7%) and stereo (94.2%), and V1_03 monocular
    (91.9%). 10 loops close, none at a wrong place;
  - global BA after a loop lowers the final ATE on 4 of the loop sequences and raises it on 4,
    by at most 8.4 mm (V1_03 stereo, 3.3 to 4.1 cm); it stays on (`loops.global_ba`), as
    ORB-SLAM3's is;
  - on the test set, 18 loops close in 14 runs, in each language. One is at a wrong place:
    advio-13's, monocular, between keyframes 13.3 m apart that looked 7.7° apart, most likely
    a place that repeats; it passed every check, its second projection at 80, the least that
    passes. Frames tracked: 209,148 in C++ and 209,019 in Rust, against B2's 208,906;
  - the languages decide alike on all 22 loop sequences and on 59 of the test set's 77 runs;
    17 of the other 18 are monocular, which the bar allows (section 10).
- **Deferred from B3's review panel**
  ([the report](../reviews/2026-10-17-feature-2b-b3-loop-closure.md), decided by evan-wes on
  2026-10-17):
  - **Global BA's damping (L2).** GTSAM's Levenberg-Marquardt damps by λ·I, absolute, from 1e-5
    up to its bound of 1e5, while a BA's Hessian holds 1e6 to 1e9 on its poses' diagonal: one
    observation just in front of a camera's image plane stops global BA at its damping limit in
    its first iteration, the map left as the essential graph made it (advio-13's). g2o, under
    ORB-SLAM, starts λ at τ times the Hessian's largest diagonal entry; GTSAM offers
    Marquardt's scaling (`setDiagonalDamping`). In the three languages, in a PR of its own
    after B3, measured on the loop sequences and the test set, since it changes decisions and
    the records (section 7).

## 12. B3b: the threads (0.3.13.0 -> 0.3.14.0)

Decided with evan-wes on 2026-10-17, following ORB-SLAM's papers and a reading of its source
(ORB-SLAM3's `Tracking.cc`, `LocalMapping.cc`, `LoopClosing.cc` and `Optimizer.cc`). Until now
every step has run in the frame that needed it, so a run repeats exactly and mapping is judged
by its load (the real-time note's rules of 2026-10-07). B2 measured that load at 1.74 to 2.63 on
the test set's five largest maps, and 3.16 to 5.89 over their busiest 10 s (section 9): there, a
mapping thread of its own falls behind. ORB-SLAM keeps up by refusing keyframes and cutting
local BA short, so on the largest maps B3b is expected to lose some accuracy; section 13
measures how much.

1. **The seams first,** in commits that change no decision, every record the same:
   - the run's records (the trajectory's placements, the lives and the closed maps: `_date`,
     `_close`, `lifetimes`, `trajectory`) leave the pipeline for `slam_it.edge.atlas`, the
     module the A2 panel proposed and B4 grows;
   - tracking, local mapping and loop closing become functions over state of their own
     (tracking's: the last frame, the motion and the loss's counters; mapping's: the map and the
     keyframes held; loop closing's: the loop being confirmed and the database), which the
     pipeline calls in turn;
   - the map is reached through one owner: the active map read, a mapping step's result
     applied, a correction applied;
   - the handshake that stops mapping, aborts global BA and releases them is written once, for
     the loop's correction and B4's merge.

   B3b then moves the calls into threads, and B4 changes what the steps do.
2. **ORB-SLAM's threads:** tracking in the caller's; local mapping, loop closing and global BA
   each in one of its own; a queue of keyframes from tracking to mapping, and one from mapping to
   loop closing.
3. **The map shared, our design: one writer, and a view published for tracking.**
   - ORB-SLAM3's sharing: tracking holds the map's update lock through its whole frame
     (`Tracking::Track`); local BA solves without it and takes it to write its results back;
     culling, triangulation and fusion run beside tracking under each keyframe's and point's own
     locks; global BA's results are carried through the spanning tree under the lock, over the
     whole map, while tracking waits. Its source shows where that sharing slips (read from the
     source, not observed): the abort flags are plain `bool`s, written and read across threads
     unlocked; `CorrectLoop` empties mapping's queue before mapping has stopped (`MergeLocal`
     waits first); an aborted global BA can still be writing its results' fields as the next one
     starts; and tracking can read keyframes and points partway through a change that spans
     several of them.
   - Ours: one thread changes the map at a time, mapping or, with mapping stopped, loop closing
     (ORB-SLAM's `RequestStop`). After making a keyframe's points, after its local BA and after
     each correction, the writer publishes a view of what tracking reads: the local map's
     keyframes and points (their poses, positions, descriptors, viewing directions and distance
     ranges), and the reference keyframe's keypoints and points. A view is swapped in whole and
     never changed after. Tracking reads only the view and takes no lock. What it writes (each
     point's counts of frames seen and found in, and the keyframes it asks for) goes to mapping as
     messages. Relocalisation, rare, reads the whole map and its database with mapping held off.
   - Python's map is already immutable, each operation returning a new one, so its view is the
     map itself, published by reference.
   - Why: tracking never waits on a lock, and no thread changes what another is reading, by
     construction. The cost: a copy per publish, a few hundred kilobytes at B2's local map (a
     median of 38 keyframes and 2,539 points; an estimate), and tracking sees a new point only
     once mapping publishes it, where ORB-SLAM's tracking sees it as it is made.
4. **The scheduling, ORB-SLAM's** (`NeedNewKeyFrame`, `LocalMapping::Run`), in our design and
   the baseline alike, so that only the sharing differs:
   - a keyframe is asked for only while mapping accepts one: a monocular map waits until mapping
     is idle; a stereo or RGB-D map interrupts the local BA and queues up to 3;
   - every keyframe queued aborts the local BA running;
   - fusion with the neighbours, local BA and keyframe culling run only while no keyframe is
     queued;
   - none is asked for while mapping is stopped or being stopped.

   `mapping.need_keyframe`'s assumption that mapping is always idle goes.
5. **Global BA in a thread of its own** (`RunGlobalBundleAdjustment`), on a copy of the map taken
   when the loop is corrected. A new loop aborts it, through a generation counter, the flags
   atomic. When it finishes, with mapping stopped, each keyframe it solved takes its result,
   each keyframe made since moves with its parent through the spanning tree, and each point with
   its reference keyframe. Section 10's "runs inline, so none needs correcting" (item 6) ends.
6. **A map lost:** tracking closes it by a message to mapping, which first finishes the keyframes
   queued for it (ORB-SLAM3's `CreateMapInAtlas` does not drain them; its queued keyframes carry
   their map), and tracking starts the next map at once. A map of 10 keyframes or fewer is reset
   only once both workers have stopped, as `ResetActiveMap` waits for them.
7. **ORB-SLAM's own scheme, in our C++,** the baseline (section 4): its locks, held where it
   holds them. Our map has no lock per keyframe or point, so where ORB-SLAM takes those, the
   baseline takes the map's. That is stricter than ORB-SLAM, and is said beside each comparison.
   ORB-SLAM3 itself, built and run here once phase B is done, compares the whole system. Its
   culling's wider set (L4) and the K2 local map, larger than its source's (L6), belong to that
   comparison, not this one, where both schemes run our steps.
8. **A setting runs the steps in order,** in one thread, as every run has until now: the
   records, the stage tests and debugging use it, and such a run repeats exactly.
9. **Frames fed at their recorded rate** when threaded, as ORB-SLAM3's examples feed them (each
   sleeps out the rest of its frame's interval), with none dropped: a late frame delays the next.
   Python, far slower than its frames, is fed as fast as it tracks.
10. **Python runs the same design under the GIL** (section 4), the readable reference held to the
    bar.
11. **The budget judge:**
    - a frame's latency is its build and track, measured as they ran;
    - mapping is measured by its queue, the keyframes refused and the local BAs aborted;
    - for runs made in order, the backlog a mapping thread would build up, by Lindley's
      recursion (L8, deferred from PR 9's panel);
    - a budget's cores are read as physical cores (L12, the same).
12. **The run format, version 8:**
    - per frame, the time tracking waited for the map;
    - per keyframe asked for, whether it was refused, and mapping's queue;
    - per local BA, whether it was aborted;
    - per global BA, whether it was aborted, or else the keyframes moved after it.
13. **Tests:**
    - the records and the stage tests run in order, as now;
    - the loop room's bar runs threaded too, in C++ and Rust, fed at its rate;
    - each handshake (stop, release, abort, the view's swap) is tested natively in each
      language;
    - layer 1 builds the C++ tests once more with ThreadSanitizer; Rust's threads are safe code.
14. **A failed correction, a question from B3's review panel** (H17;
    [the report](../reviews/2026-10-17-feature-2b-b3-loop-closure.md)): a GTSAM failure inside a
    loop's correction propagates out of the pipeline's loop closing and ends the whole run.
    Whether a failed correction should instead be recorded and the loop skipped is decided with
    B3b's design, where loop closing runs in a thread of its own.

**Deferred for this PR** (decided by evan-wes on 2026-10-18: no long runs for this PR; tracked in
the ladder's open items):
- **Section 13's measurements** are not run for this PR: the latency bar, the accuracy threaded
  on the loop sequences and the test set, and our design against ORB-SLAM's scheme in C++. The
  judge and the scripts that make them are built (item 11: `scripts/measure_testset.py`'s
  `--threaded`, `--locks` and `--budget`, and `scripts/measure_sharded.py`'s `--cores`).
- **A flaky test, not investigated.** The loop room's bar threaded in C++, its TUM monocular
  case (`tests/orchestration/test_loop_room.py`'s `test_every_backend_closes_the_loop_room`):
  in one of four runs the final ATE was 8.99 cm against the 2 cm bound, and 1.54 to 1.62 cm in
  the other three, on a busy machine; threaded runs do not repeat exactly. It was to be run about
  20 times on a quiet machine before and after the commit that publishes a view of what
  tracking reads in place of the whole map (item 3), and the failure rates compared; that was
  not done, so whether the commit changed it is not known.
- **A failed correction (item 14) is still open,** with no decision yet: whether a GTSAM failure
  inside a loop's correction should be recorded and the loop skipped.

## 13. Measurements (B3b)

Once, on the finished branch, C++ and Rust (section 11's rule).
- **The latency bar** (decided with evan-wes on 2026-10-17): at least 99% of the frames built
  and tracked within their deadline (the median frame interval), C++ and Rust each, each run
  alone on cores of its own; the late frames are listed.
- **Accuracy, threaded:** the loop sequences and the test set, threaded and fed in real time,
  against B3's runs made in order. The bar still holds: the final ATE within 1.5 times the
  published figure, at least 95% of the frames tracked, no loop at a wrong place. Also measured:
  the keyframes made and refused, the local BAs aborted, and the loops closed. Each run gets
  cores of its own, so only a few run at a time, and the test set takes its own length in wall
  time.
- **Our design against ORB-SLAM's scheme,** in C++ on the same runs: the latency (median, 99th
  percentile, worst), the time tracking waited, and the accuracy.

## 14. B4: merging maps (0.3.13.0 -> 0.3.14.0)

Decided with evan-wes on 2026-10-17, following ORB-SLAM3's paper (its section VI) and a reading
of its source (`Atlas.cc`, `LoopClosing.cc`'s `MergeLocal`, `KeyFrameDatabase.cc` and
`Optimizer.cc`). Where the paper and the source differ, each item says which it follows.

1. **The atlas** (`slam_it.edge.atlas`, from B3b's seams) keeps the active map and every map
   kept, whole: keyframes with their keypoints and descriptors, points, observations, the
   spanning tree and the links. Each map has its own database, and the maps are numbered in the
   order they started, as ORB-SLAM3's `Atlas` keeps them.
   - It replaces the closed maps' records, and the lost map's strong links computed in the frame
     it is lost on (kept until now, decided with evan-wes on 2026-10-16).
   - A map's keyframes and points carry their run ids rather than a base, since a merge brings
     one map's into another's.
2. **Relocalisation in every map** (L7), as ORB-SLAM3's paper describes it; its source searches
   the active map alone.
   - Each map's database finds its own candidates, its words counted within the map. All maps'
     candidates are then tried best score first; the scores are comparable under the one
     vocabulary.
   - ORB-SLAM3's source counts words across the atlas before it drops other maps' candidates,
     so a strong match in another map can raise the cut past every candidate in the active one.
   - Found in another map, tracking goes on in it, and the lost map is kept.
3. **Detecting a merge, ORB-SLAM3's** (`NewDetectCommonRegions`'s merge branch). It runs on each
   keyframe beside loop detection, by the same stages (section 10, item 2, with B3's
   departures):
   - up to 3 candidates from the other maps (`DetectNBestCandidates`); a map of fewer than 12
     keyframes searches none;
   - its own merge being confirmed, as a loop is;
   - a merge and a loop found on one keyframe: the merge goes first, and the loop is dropped
     (`Run`).

   Unlike ORB-SLAM3's source, a confirmation resets the merge's misses as it resets the loop's,
   so it takes two misses in a row to drop a merge. A monocular merge's Sim(3) carries the two
   maps' scale ratio, with no bound on it (ORB-SLAM3's 0.9 to 1.1 is for inertial maps). The
   loop's searches take two maps: the new keyframe's and the matched one's.
4. **The merge** (`MergeLocal`, its correction the paper's):
   - mapping is stopped and global BA aborted (B3b's handshake);
   - the matched map, the older, survives with its frame, scale and number. The active map's
     keyframes and points move into it by the merge's Sim(3), each keyframe by its pose relative
     to the new one and each point with its reference keyframe, keeping their run ids. The
     merged map becomes the active one. ORB-SLAM3 hands the survivor the active map's number
     (`ChangeId`, marked "TODO for debug"); not here;
   - the active map's spanning tree is re-rooted at the new keyframe and hung under the matched
     one. Its first keyframe becomes an ordinary one, so no root is renumbered and
     `covisibility.cull`'s rule stands (A15);
   - the welding window: each of the two keyframes and its 25 best links, widened through its
     members' 12 best links, up to 5 rounds, while it holds fewer (ORB-SLAM3's
     `numTemporalKFs`);
   - the matched side's points are fused into the active side's keyframes (`SearchAndFuse`:
     4 pixels, `TH_LOW`), each replacing the point a keypoint had;
   - **welding BA, the paper's:** both sides' keyframes in the window and their points
     optimised, with the matched map's keyframes outside it that see those points fixed; a Huber
     loss, 5 iterations, the outliers dropped, then 10 more. ORB-SLAM3's source fixes the matched
     side's window instead;
   - **the essential graph over the whole merged map,** the window fixed, in every mode, as the
     paper does it. ORB-SLAM3's source covers only the rest of the active map, and only for
     stereo and RGB-D; it moves a monocular map by a single similarity, so its drift is never
     spread;
   - **global BA after,** as after a loop (section 10, item 6), in B3b's thread; ORB-SLAM3's
     source runs it only where one was interrupted. `loops.global_ba` turns it off for both;
   - the two keyframes are recorded as merged (`AddMergeEdge`), and mapping is released.
5. **Sessions:** a run over several sequences in turn, with a new map at each one's start, as
   ORB-SLAM3's `ChangeDataset` starts one (a map of fewer than 12 keyframes reset, others kept).
   The input takes a list of sequences.
6. **The loss window, measured both ways:** B1's 3 s stays the default. A new map at once is
   measured beside it on the test set (section 15), and decided with evan-wes on the numbers.
7. **The run format, version 9, and the bundle's, version 5:**
   - each keyframe and point records the map it ends in; each frame records the map it ends in
     (the final trajectory's) and the map it was tracked in (the as-tracked one's);
   - a run's maps no longer only rise, and a map's frames need not be contiguous;
   - the merges are kept (`keyframe_merges.npy`, `(E, 2)` int64), and each is recorded in the
     diagnostics as a loop is: the maps, the keyframes, the Sim(3), the points fused, and the
     results of the welding BA, the graph and global BA;
   - each map is still aligned on its own. The final trajectory holds every kept map's frames,
     where ORB-SLAM3's saves only its largest map's;
   - the viewer draws the merges.
8. **Tests:**
   - the merge room: the loop room's circle with black frames partway, longer than the loss
     window, so the track is lost and a new map starts, and the path passing its start again, so
     the maps merge. Every language ends in one map, merged at the right place, with its final
     ATE and its points' distance to the walls within bounds set from the first runs;
   - the stage comparisons with Python's reference, for the two-map searches and the merge;
   - the records, one per language, regenerated.
9. **Not in B4:** reifyscene's adapter for several maps (A10, which lives in reifyscene);
   merging inertial maps (`MergeLocal2`, milestone 6).
10. **Sim(3) as a group in rung 0, from B3's review panel** (A5, with H3;
    [the report](../reviews/2026-10-17-feature-2b-b3-loop-closure.md), decided by evan-wes on
    2026-10-17), which B4's review panel checks: Sim(3)'s group operations (inverse, scale,
    rigid part, compose) in one rung-0 module in each language, the RANSAC module named for what
    it estimates; `graphs` and `edge.loops` use it, one validated split of a similarity (so a
    similarity `loops.detect` accepts, `graphs` accepts too), and `so3.hat` in place of
    `graphs._hat`. B4's merge corrects by a Sim(3) with the same operations, and the scale has
    two definitions today (`cbrt(det)` and a column's norm), which agree only on exact inputs.

## 15. Measurements (B4)

Once, on the finished branch, C++ and Rust, threaded and fed in real time (section 13).
- **Multi-session EuRoC, the bar** (decided with evan-wes on 2026-10-17): MH_01 to MH_03, MH_01
  to MH_05 and V1_01 to V1_03, monocular and stereo, each room's sequences in one run, aligned
  once to the ground truth, as the paper aligns them. The bar:
  - the ATE within 1.5 times ORB-SLAM3's (its Table V, a median of 5 runs): monocular 3.0, 5.8
    and 5.8 cm; stereo 2.8, 4.0 and 2.7 cm;
  - every session ending in one map;
  - at least 95% of the frames tracked;
  - no loop or merge at a wrong place.

  V2_01 to V2_03 are not run: that room is not downloaded, and the disk is short.
- **TUM's fr2_360_kidnap,** RGB-D and monocular: whether the track is found again after the
  kidnap, by relocalisation or by a merge, and the ATE. There is no published figure.
- **7-Scenes across sequences:** chess, fire, heads and stairs, each scene's sequences in one
  run: how many sessions relocalise into the first's map or merge with it, and the ATE. There is
  no published figure. The scenes are unpacked first, their size checked.
- **The loss window:** the test set with 3 s and with a new map at once: the share tracked, the
  maps, the merges and the final ATE.
- **The test set and the loop sequences,** against B3b's.
