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
  - PnP RANSAC (MLPnP: confidence 0.99, at least 10 inliers, at most 300 iterations, a minimal
    set of 6, the chi-square threshold 5.991), run 5 iterations at a time across the
    candidates;
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
   - its files, `.npy` arrays all three languages read as they read the run format: each node's
     descriptor (`(N, 32)` uint8), its parent, the first of its children (children are
     consecutive), each leaf's weight (float64), and a `vocabulary.json` naming its k, levels,
     training set, seed and the SHA-256 of each array;
   - the transform: a descriptor descends from the root, at each level to the nearest child by
     Hamming distance (the lowest index among equals), to its word; a frame's words form its
     vector, weighted and L1-normalised; its features are grouped by their node at level 2
     (the direct index; ORB-SLAM's level 2 of 6, kept at level 2 of 5);
   - the score: DBoW2's L1 score, 1 - |v - w|_1 / 2;
   - the default is the committed 5-level vocabulary; a run's `--vocabulary` (`vo.run`'s
     `vocabulary`) names another folder, whose arrays must match their recorded checksums, and
     the run's settings record its name and checksums;
   - a vocabulary's words mean something only for the descriptor they were trained on: its
     `vocabulary.json` records the descriptor's settings (the patch, the smoothing, the test
     pattern), and a run whose descriptor differs refuses it, rather than scoring against words
     that no longer mean anything. The feature count and the pyramid do not change a
     descriptor, so they are not checked.
2. **The training tool** (`scripts/train_vocabulary.py`, Python only): ORB features as the
   pipeline extracts them, from a training set's images, hierarchical k-means with k-means++
   seeding from a fixed seed and the bitwise majority as a centre, weights from the training
   images' document frequencies. Deterministic for a seed: a rebuild gives the same checksums.
   Three vocabularies, each from 10,000 images drawn with a fixed seed:

   | Vocabulary | Trained on | Levels | Kept |
   |---|---|---|---|
   | ORB-SLAM's kind | Bovisa 2008-09-01, the front camera | 6 | `../vocabularies/` |
   | A phone's | ARKitScenes' training fold | 6 | the same |
   | The default | ARKitScenes' training fold | 5 | committed, `data/vocabulary/` |

   If Bovisa's serves the test set better, the default is retrained from it.
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
   map), is relocalised against the whole active map. A run is recently lost from its first
   failed frame: it is tracked again on a frame that succeeds, and lost, its map closed as A2
   closes it, once `relocalize_seconds` (3.0) have passed since that frame with no success. A
   map of `reset_keyframes` or fewer is lost after `lost_frames` (5) failures instead, as
   section 6 has it. The pipeline's own state stays tracking or not initialised between
   frames, the loss flagged on the frame's step (the A2 panel's A8).
6. **The run format,** still version 4: a frame relocalised is placed by `relocalization`, and
   its diagnostics say how relocalisation went (the candidates, each one's matches and inliers,
   the outcome), null where it did not run.
7. **The measure, the bake and the status page:** relocalisations counted per run beside the
   losses; the viewer's timeline marks a relocalised frame; the summary says how often a lost
   track was found again.
8. **Parity:** the golden record gains a room where the track is lost and found again: the lost
   room's blank frames end where the camera sees what it saw before, so the next frame
   relocalises in map 0 instead of starting map 1. Its rooms that never lose the track keep
   their records unchanged.
9. **Rust's cost**, as the ladder decided: measured after this PR, and Rust kept or dropped.
10. **The two-view sample limit,** deferred from A2's measurement: a failed monocular start
   takes up to 0.43 s on ADVIO, every late frame of the test set, where rung 0's two-view RANSAC
   draws up to 10,000 samples per model (`twoview.Options.max_iterations`) and ORB-SLAM's
   `TwoViewReconstruction` 200. Its own commit, first, since it changes every monocular start,
   measured apart from relocalisation.

## 4. B2 to B4, in outline

Each gets its own section here before its PR, its forks put to the owner then.
- **B2, the covisibility graph:** the local map from the keyframes that see the frame's points
  and their best neighbours (ORB-SLAM's K1 and K2, up to 80 keyframes), local BA over the
  covisible keyframes, culling by covisibility; the essential graph kept for B3. It changes every
  run's decisions, so the golden record is regenerated whole and the test set rerun against B1's.
- **B3, loop closure:** inline, not in a thread, as the ladder decided; the frame it closes on
  measured against the deadline, a thread only if it misses it. Measured on TUM and EuRoC's
  loops.
- **B4, merging maps:** the atlas the A2 panel deferred (`edge.atlas`: a kept map's descriptors
  and observations kept whole, the pipeline owning the trajectory record), and maps welded where
  the database recognises one in another.

## 5. Measurements (B1)

- **The two-view sample limit first,** the monocular runs at seeds 0, 1 and 2 against A2's:
  the starts, the share tracked and the late frames.
- **The test set at seeds 0, 1 and 2**, against A2's: the share tracked, the maps per run, the
  losses and relocalisations, the latency and late frames, with the default vocabulary; then,
  at seed 0, the two 6-level ones, Bovisa's and ARKitScenes', so that the training set and the
  size are each read apart.
- **Relocalisation across sequences** (7-Scenes' four scenes, section 6): a scene's sequences
  share a world frame, so a run of a training sequence followed by a test sequence must
  relocalise the test frames in the training sequence's map. The share relocalised and their
  error against the ground truth, monocular and RGB-D.
- **TUM's `fr2_360_kidnap`**, whose camera is covered and carried elsewhere, and ZJU's black
  frames: whether each relocalises in its first map.

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
  offline CI before the merge. Then Rust's cost.
- **B2, B3 and B4** after it, each designed here first.
- **Not in 2B:** the IMU (milestone 6); the learned rungs.
