//! Relocalisation: the Rust core's mirror of `slam_it.edge.relocalization` (ORB-SLAM's
//! `Tracking::Relocalization`), whose docstring states the steps: the database's candidates,
//! matching by word, PnP RANSAC as ORB-SLAM sets `MLPnP`'s, the pose optimised and topped up by
//! searches by projection, 50 inliers to place the frame.

use nalgebra::Vector3;
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::matching::{self, Match, Projection, SearchOptions};
use slam_it_foundations::p3p;
use slam_it_foundations::ransac;
use slam_it_foundations::vocabulary::Vocabulary;

use crate::database::{Database, Words, candidates, words_of};
use crate::frames::{Camera, Frame, Tracked, check_setup, search};
use crate::map::{Map, camera_centre, in_distance_range, predict_level, to_camera};
use crate::options::Options;
use crate::tracking::{PnpRecord, RecordsPnp, optimize_pose, pnp_kept};

/// How far one candidate got (`slam_it.edge.relocalization.CandidateSummary`); each count
/// `None` where it stopped first.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CandidateSummary {
    /// The candidate's id in the map.
    pub keyframe: i64,
    /// Its group's accumulated score.
    pub score: f64,
    /// The frame's keypoints matched to its points by word, with a bearing in front.
    pub matches: usize,
    /// RANSAC's samples, if it ran and found a pose.
    pub iterations: Option<usize>,
    /// Why RANSAC stopped, if it found a pose.
    pub termination: Option<ransac::Termination>,
    /// Whether RANSAC ran and found no pose.
    pub degenerate: bool,
    /// RANSAC's inliers.
    pub inliers: Option<usize>,
    /// The inliers the first optimisation kept.
    pub refined: Option<usize>,
    /// The inliers after the searches by projection: the last optimisation's.
    pub found: Option<usize>,
}

impl RecordsPnp for CandidateSummary {
    fn record_pnp(&mut self, record: PnpRecord) {
        record.write(
            &mut self.degenerate,
            &mut self.iterations,
            &mut self.termination,
            &mut self.inliers,
        );
    }
}

/// How relocalising a frame went (`slam_it.edge.relocalization.RelocalizationSummary`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RelocalizationSummary {
    /// The frame's words with a weight.
    pub words: usize,
    /// The keyframes that shared a word with it.
    pub sharing: usize,
    /// Those scored.
    pub scored: usize,
    /// The candidates tried, in order; the last placed the frame if any did.
    pub candidates: Vec<CandidateSummary>,
    /// Whether the frame was placed.
    pub placed: bool,
}

/// Relocalisation's result (`slam_it.edge.relocalization.Relocalized`).
#[derive(Debug, Clone, PartialEq)]
pub struct Relocalized {
    /// The frame placed, with its inliers, if a candidate placed it.
    pub tracked: Option<Tracked>,
    /// How it went.
    pub summary: RelocalizationSummary,
    /// The id of the map point each keypoint matched in the last candidate tried, by word or by
    /// its searches by projection; -1 where none.
    pub matched: Vec<i64>,
}

/// One candidate tried: the frame if it placed it, how far it got, and its matches.
struct Attempt {
    /// The frame placed, if it was.
    tracked: Option<Tracked>,
    /// How far it got.
    summary: CandidateSummary,
    /// Each keypoint's match by word or by projection, or -1.
    matched: Vec<i64>,
}

/// A keyframe's row, refusing one the map does not hold.
///
/// # Arguments
///
/// * `m` - The map.
/// * `keyframe` - Its id.
///
/// # Errors
///
/// `InvalidInput` if the map does not hold it.
fn row_of(m: &Map, keyframe: i64) -> Result<usize> {
    match m.keyframe_row(keyframe) {
        Some(k) => Ok(k),
        None => fail(
            ErrorKind::InvalidInput,
            format!("keyframe {keyframe} is not in the map"),
        ),
    }
}

/// How many keypoints matched a point.
///
/// # Arguments
///
/// * `points` - Each keypoint's point id, or -1.
///
/// # Returns
///
/// The matched ones.
fn count_matched(points: &[i64]) -> usize {
    points.iter().filter(|&&p| p >= 0).count()
}

/// A search's matches added to a frame's.
///
/// # Arguments
///
/// * `points` - Each keypoint's point id, or -1.
/// * `added` - The search's, or -1.
///
/// # Returns
///
/// The search's where it matched, the frame's elsewhere.
fn merged(points: &[i64], added: &[i64]) -> Vec<i64> {
    points
        .iter()
        .zip(added)
        .map(|(&p, &a)| if a >= 0 { a } else { p })
        .collect()
}

/// The inliers a candidate's pose needs, and the most samples its RANSAC draws
/// (`slam_it.edge.relocalization.ransac_budget`, `MLPnP`'s `SetRansacParameters`).
///
/// # Arguments
///
/// * `matches` - The candidate's matches by word.
/// * `options` - `relocalization.min_inliers`, `relocalization.inlier_ratio`,
///   `relocalization.confidence` and `relocalization.ransac_iterations`.
///
/// # Returns
///
/// `(needed, samples)`.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "match counts are far below 2^52; the floor and ceiling are small and positive"
)]
pub fn ransac_budget(matches: usize, options: &Options) -> (usize, usize) {
    let needed = options
        .relocalization
        .min_inliers
        .max((options.relocalization.inlier_ratio * matches as f64).floor() as usize)
        .max(p3p::SAMPLE_SIZE);
    if needed >= matches {
        return (needed, 1);
    }
    let ratio = options
        .relocalization
        .inlier_ratio
        .max(needed as f64 / matches as f64);
    let bound =
        ransac::required_iterations(ratio, p3p::SAMPLE_SIZE, options.relocalization.confidence);
    let samples = bound.ceil() as usize;
    (
        needed,
        samples.min(options.relocalization.ransac_iterations).max(1),
    )
}

/// The settings of one search by projection.
#[derive(Debug, Clone, Copy)]
pub struct SearchWindow {
    /// The window's half-width, in pixels at level 0.
    pub radius: f64,
    /// The largest Hamming distance a match may have.
    pub distance: i64,
}

/// Searches a keyframe's points in a frame by projection
/// (`slam_it.edge.relocalization.search_keyframe`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `tracked` - Its pose, and the keypoints already matched, never candidates.
/// * `keyframe` - The keyframe's id.
/// * `skip` - The ids of points not searched.
/// * `window` - The window and the distance.
/// * `options` - The distance margins.
///
/// # Returns
///
/// The id of the point each keypoint matched by this search, -1 where none.
///
/// # Errors
///
/// `InvalidInput` for a keyframe the map does not hold.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::cast_possible_truncation,
    reason = "levels are at most MAX_LEVELS"
)]
#[allow(
    clippy::too_many_arguments,
    reason = "the map, its camera, the frame, the keyframe, the search"
)]
pub fn search_keyframe(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    tracked: &Tracked,
    keyframe: i64,
    skip: &[i64],
    window: SearchWindow,
    options: &Options,
) -> Result<Vec<i64>> {
    let k = row_of(m, keyframe)?;
    let mut skipped = skip.to_vec();
    skipped.sort_unstable();
    let centre = camera_centre(&tracked.pose);
    let mut points = Vec::new();
    let mut gaps = Vec::new();
    let mut projections = Vec::new();
    for row in m.kf_start[k]..m.kf_start[k + 1] {
        let id = m.kp_point[row];
        if id < 0 || skipped.binary_search(&id).is_ok() {
            continue;
        }
        let Some(p) = m.point_row(id) else {
            continue;
        };
        let xyz = m.pt_xyz[p];
        let at = to_camera(&tracked.pose, &xyz);
        if !(at.z > 0.0) {
            continue;
        }
        let pixel = camera.project(&at);
        let gap = (xyz - centre).norm();
        if !(camera.inside(&pixel) && in_distance_range(m, p, gap, options)) {
            continue;
        }
        points.push(p);
        gaps.push(gap);
        projections.push(Projection {
            xy: [pixel.x, pixel.y],
            radius: 0.0,
            min_level: 0,
            max_level: 0,
            descriptor: m.pt_descriptor[p],
            angle: m.kp_angle[row],
            right: f64::NAN,
        });
    }
    let levels = predict_level(m, &points, &gaps)?;
    for (projection, &level) in projections.iter_mut().zip(&levels) {
        projection.radius = window.radius * m.scale_factor.powf(level as f64);
        projection.min_level = level as i64 - 1;
        projection.max_level = level as i64 + 1;
    }
    let taken: Vec<bool> = tracked.points.iter().map(|&p| p >= 0).collect();
    let found = search(
        frame,
        &projections,
        &SearchOptions {
            max_distance: window.distance,
            ratio: 1.0,
            check_orientation: true,
        },
        &taken,
    )?;
    let mut out = vec![-1; frame.size()];
    for found in &found {
        out[found.second] = m.pt_id[points[found.first]];
    }
    Ok(out)
}

/// Searches the candidate's other points twice, optimising when they bring enough inliers
/// (`slam_it.edge.relocalization._project_more`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `keyframe` - The candidate's id.
/// * `refined` - The frame after the first optimisation.
/// * `found` - The points RANSAC's inliers matched, which the first search skips.
/// * `options` - The settings.
///
/// # Returns
///
/// The frame as last optimised, and every match before that optimisation.
///
/// # Errors
///
/// A search's or an optimisation's.
fn project_more(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    keyframe: i64,
    refined: &Tracked,
    found: &[i64],
    options: &Options,
) -> Result<(Tracked, Vec<i64>)> {
    let needed = options.tracking.recovery_tracked;
    let wide = SearchWindow {
        radius: options.relocalization.window,
        distance: options.matching.th_high,
    };
    let added = search_keyframe(m, camera, frame, refined, keyframe, found, wide, options)?;
    let all = merged(&refined.points, &added);
    if refined.inliers() + count_matched(&added) < needed {
        return Ok((refined.clone(), all));
    }
    let start = Tracked {
        pose: refined.pose,
        points: all.clone(),
    };
    let again = optimize_pose(m, camera, frame, &start, options)?;
    let good = again.inliers();
    if !(options.relocalization.narrow_inliers < good && good < needed) {
        return Ok((again, all));
    }
    // The second optimisation's outliers keep their matches until the last (ORB-SLAM's).
    let held = Tracked {
        pose: again.pose,
        points: all.clone(),
    };
    let skip: Vec<i64> = all.iter().copied().filter(|&p| p >= 0).collect();
    let narrow_window = SearchWindow {
        radius: options.relocalization.narrow_window,
        distance: options.relocalization.narrow_distance,
    };
    let narrow = search_keyframe(
        m,
        camera,
        frame,
        &held,
        keyframe,
        &skip,
        narrow_window,
        options,
    )?;
    if good + count_matched(&narrow) < needed {
        return Ok((again, all));
    }
    let last = Tracked {
        pose: held.pose,
        points: merged(&all, &narrow),
    };
    let optimized = optimize_pose(m, camera, frame, &last, options)?;
    Ok((optimized, last.points))
}

/// A candidate's matches by word, with a bearing in front.
///
/// # Arguments
///
/// * `m` - The map.
/// * `frame` - The frame.
/// * `rows` - The candidate's keypoint rows that observe a point.
/// * `nodes` - Each row's node.
/// * `frame_nodes` - Each of the frame's keypoints' node.
/// * `options` - `matching.th_low` and `relocalization.ratio`.
///
/// # Returns
///
/// `(row index, keypoint)` matches.
///
/// # Errors
///
/// Matching's.
fn match_by_word(
    m: &Map,
    frame: &Frame,
    rows: &[usize],
    nodes: &[usize],
    frame_nodes: &[usize],
    options: &Options,
) -> Result<Vec<Match>> {
    let descriptors: Vec<_> = rows.iter().map(|&row| m.kp_descriptor[row]).collect();
    let angles: Vec<usize> = rows.iter().map(|&row| m.kp_angle[row]).collect();
    let frame_angles: Vec<usize> = frame.features.keypoints.iter().map(|k| k.angle).collect();
    let found = matching::search_by_bow(
        &descriptors,
        nodes,
        &frame.features.descriptors,
        frame_nodes,
        &angles,
        &frame_angles,
        &matching::Options {
            max_distance: options.matching.th_low,
            ratio: options.relocalization.ratio,
            check_orientation: true,
        },
    )?;
    Ok(found
        .into_iter()
        .filter(|pair| frame.bearings[pair.second].z > 0.0)
        .collect())
}

/// Tries to place the frame through one candidate (`slam_it.edge.relocalization._try`, the
/// module docstring's steps 2 to 4).
///
/// # Arguments
///
/// * `m` - The map.
/// * `db` - The database.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `frame_nodes` - Each of the frame's keypoints' node.
/// * `candidate` - The candidate's id and score.
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` for a candidate the map does not hold or whose features are not the
/// database's.
fn try_candidate(
    m: &Map,
    db: &Database,
    camera: &Camera,
    frame: &Frame,
    frame_nodes: &[usize],
    candidate: (i64, f64),
    options: &Options,
) -> Result<Attempt> {
    let (keyframe, score) = candidate;
    let k = row_of(m, keyframe)?;
    let (start, end) = (m.kf_start[k], m.kf_start[k + 1]);
    let nodes = &db.keyframes[&keyframe].nodes;
    if nodes.len() != end - start {
        return fail(
            ErrorKind::InvalidInput,
            format!("database: keyframe {keyframe}'s features are not the map's"),
        );
    }
    let rows: Vec<usize> = (start..end).filter(|&row| m.kp_point[row] >= 0).collect();
    let row_nodes: Vec<usize> = rows.iter().map(|&row| nodes[row - start]).collect();
    let pairs = match_by_word(m, frame, &rows, &row_nodes, frame_nodes, options)?;
    let mut matched = vec![-1; frame.size()];
    for pair in &pairs {
        matched[pair.second] = m.kp_point[rows[pair.first]];
    }
    let mut summary = CandidateSummary {
        keyframe,
        score,
        matches: pairs.len(),
        ..CandidateSummary::default()
    };
    let unplaced = |summary, matched| {
        Ok(Attempt {
            tracked: None,
            summary,
            matched,
        })
    };
    let (needed, samples) = ransac_budget(pairs.len(), options);
    if pairs.len() < options.relocalization.min_matches.max(needed) {
        return unplaced(summary, matched);
    }
    let Some(world) = pairs
        .iter()
        .map(|pair| m.point_row(matched[pair.second]).map(|p| m.pt_xyz[p]))
        .collect::<Option<Vec<Vector3<f64>>>>()
    else {
        return fail(
            ErrorKind::InvalidInput,
            "map: a keypoint observes a point not in it",
        );
    };
    let rays: Vec<Vector3<f64>> = pairs
        .iter()
        .map(|pair| frame.bearings[pair.second])
        .collect();
    let settings = ransac::Options {
        threshold: options.tracking.pnp_threshold / camera.focal(),
        confidence: options.relocalization.confidence,
        max_iterations: i64::try_from(samples).unwrap_or(i64::MAX),
        seed: options.seed,
        ..ransac::Options::default()
    };
    let Some(consensus) = pnp_kept(&world, &rays, &settings, needed, &mut summary)? else {
        return unplaced(summary, matched);
    };
    let mut sample = vec![-1; frame.size()];
    for (pair, &inlier) in pairs.iter().zip(&consensus.inliers) {
        if inlier {
            sample[pair.second] = matched[pair.second];
        }
    }
    let found: Vec<i64> = sample.iter().copied().filter(|&p| p >= 0).collect();
    let start_pose = Tracked {
        pose: consensus.model,
        points: sample,
    };
    let refined = optimize_pose(m, camera, frame, &start_pose, options)?;
    summary.refined = Some(refined.inliers());
    if refined.inliers() < options.relocalization.min_inliers {
        return unplaced(summary, matched);
    }
    let mut last = refined;
    if last.inliers() < options.tracking.recovery_tracked {
        let (searched, all) = project_more(m, camera, frame, keyframe, &last, &found, options)?;
        last = searched;
        matched = merged(&matched, &all);
    }
    summary.found = Some(last.inliers());
    Ok(Attempt {
        tracked: (last.inliers() >= options.tracking.recovery_tracked).then_some(last),
        summary,
        matched,
    })
}

/// Places a frame in the map through the keyframes that look like it
/// (`slam_it.edge.relocalization.relocalize`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `db` - Its keyframe database, holding every keyframe of the map.
/// * `vocabulary` - The vocabulary the database's words are of.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `options` - The settings.
///
/// # Returns
///
/// The frame placed with `tracking.recovery_tracked` inliers or more, or none; how the search went;
/// each keypoint's match in the last candidate tried.
///
/// # Errors
///
/// `InvalidInput` for a candidate the map does not hold or whose features are not the
/// database's, or an invalid camera or settings.
pub fn relocalize(
    m: &Map,
    db: &Database,
    vocabulary: &Vocabulary,
    camera: &Camera,
    frame: &Frame,
    options: &Options,
) -> Result<Relocalized> {
    check_setup(camera, options)?;
    let Words {
        vector,
        nodes: frame_nodes,
    } = words_of(vocabulary, &frame.features.descriptors)?;
    let found = candidates(db, m, &vector, options)?;
    let mut out = Relocalized {
        tracked: None,
        summary: RelocalizationSummary {
            words: vector.ids.len(),
            sharing: found.sharing,
            scored: found.scored,
            ..RelocalizationSummary::default()
        },
        matched: vec![-1; frame.size()],
    };
    for (&keyframe, &score) in found.keyframes.iter().zip(&found.scores) {
        let attempt = try_candidate(
            m,
            db,
            camera,
            frame,
            &frame_nodes,
            (keyframe, score),
            options,
        )?;
        out.summary.candidates.push(attempt.summary);
        out.matched = attempt.matched;
        if attempt.tracked.is_some() {
            out.tracked = attempt.tracked;
            out.summary.placed = true;
            return Ok(out);
        }
    }
    Ok(out)
}
