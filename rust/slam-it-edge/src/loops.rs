//! Loop closing: the Rust core's mirror of `slam_it.edge.loops` (ORB-SLAM3's `LoopClosing`),
//! whose docstring states the method: detection (`NewDetectCommonRegions`), a loop being
//! confirmed carried from keyframe to keyframe, and the correction (`CorrectLoop`) with its
//! essential graph and global bundle adjustment, and where it departs from ORB-SLAM3's source;
//! and merging: another map's place found by detection's stages, each search taking the map
//! searched as well as the keyframe's own, and the active map brought into it, welded, and the
//! weld spread (ORB-SLAM3's `MergeLocal`, its correction as its paper gives it).
//!
//! Similarities are ORB-SLAM's `S_cw`, `[[s R, t], [0, 1]]`; a keyframe's own pose is one with
//! `s = 1`. A search projects through the rigid `(R, t / s)`.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use nalgebra::{Matrix4, Vector2, Vector3};
use slam_it_foundations::camera::MIN_DEPTH;
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::graphs;
use slam_it_foundations::matching;
use slam_it_foundations::nls::Stop;
use slam_it_foundations::orb::{self, Descriptor};
use slam_it_foundations::ransac;
use slam_it_foundations::sim3;

use crate::covisibility;
use crate::database::{self, Database, MapRef};
use crate::frames::Camera;
use crate::map::{self, Map, camera_centre, keyframe_row, predict_level, to_camera, viewing_cos};
use crate::options::{Mode, Options};
use crate::residuals::{chi2, sigma2, threshold};

// --- similarities --------------------------------------------------------------------------

/// A similarity's scale: the length of a column of `s R` (`slam_it.edge.loops.scale_of`).
///
/// # Arguments
///
/// * `similarity` - `[[s R, t], [0, 1]]`.
///
/// # Returns
///
/// `s`.
#[must_use]
pub fn scale_of(similarity: &Matrix4<f64>) -> f64 {
    similarity.fixed_view::<3, 1>(0, 0).norm()
}

/// The rigid pose a similarity projects through, `[[R, t / s], [0, 1]]`
/// (`slam_it.edge.loops.rigid_of`, ORB-SLAM's `SE3(R, t / s)`).
///
/// # Arguments
///
/// * `similarity` - `S_cw`.
///
/// # Returns
///
/// `T_cw`, the same camera unscaled.
#[must_use]
pub fn rigid_of(similarity: &Matrix4<f64>) -> Matrix4<f64> {
    let s = scale_of(similarity);
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(similarity.fixed_view::<3, 3>(0, 0) / s));
    out.fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&(similarity.fixed_view::<3, 1>(0, 3) / s));
    out
}

/// The motion from one pose to another: `T_to_w T_frm_w⁻¹`.
///
/// # Arguments
///
/// * `to` - The pose arrived at.
/// * `frm` - The pose left.
///
/// # Returns
///
/// The motion.
fn relative(to: &Matrix4<f64>, frm: &Matrix4<f64>) -> Matrix4<f64> {
    to * sim3::inverse(frm)
}

/// Carries a point by the top three rows of a transform: `A p + b`.
///
/// # Arguments
///
/// * `transform` - The transform.
/// * `point` - The point.
///
/// # Returns
///
/// The point carried.
fn carried(transform: &Matrix4<f64>, point: &Vector3<f64>) -> Vector3<f64> {
    transform.fixed_view::<3, 3>(0, 0) * point + transform.fixed_view::<3, 1>(0, 3)
}

// --- checks --------------------------------------------------------------------------------

/// Refuses a keyframe row out of range.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The row.
/// * `name` - The argument's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn check_keyframe(m: &Map, k: usize, name: &str) -> Result<()> {
    if k >= m.keyframes() {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected a keyframe row, got {k}"),
        );
    }
    Ok(())
}

/// Refuses point rows out of range.
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The rows.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn check_points(m: &Map, rows: &[usize]) -> Result<()> {
    if rows.iter().any(|&p| p >= m.points()) {
        return fail(ErrorKind::InvalidInput, "points: expected point rows");
    }
    Ok(())
}

/// Refuses keypoint-by-keypoint ids that are not one per keypoint of a keyframe.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `ids` - The ids.
/// * `name` - The argument's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for another length, or an id below -1.
fn check_keypoint_ids(m: &Map, k: usize, ids: &[i64], name: &str) -> Result<()> {
    if ids.len() != m.kf_start[k + 1] - m.kf_start[k] || ids.iter().any(|&id| id < -1) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected one id per keypoint of keyframe row {k}"),
        );
    }
    Ok(())
}

/// How many keypoints a search matched.
///
/// # Arguments
///
/// * `found` - Each keypoint's point id, or -1.
///
/// # Returns
///
/// The matched ones.
fn matched_count(found: &[i64]) -> usize {
    found.iter().filter(|&&id| id >= 0).count()
}

// --- searches ------------------------------------------------------------------------------

/// Points that project into a keyframe: their rows, pixels and predicted levels.
struct InView {
    /// The points' rows.
    rows: Vec<usize>,
    /// Their pixels.
    pixels: Vec<Vector2<f64>>,
    /// Their predicted levels.
    levels: Vec<usize>,
}

/// The points that project into a camera through a similarity, as ORB-SLAM checks them
/// (`slam_it.edge.loops._in_view`): in front of it, in the image, within the distances they are
/// matchable from (no margins), and seen within `tracking.view_cos_limit` of their mean
/// viewing direction.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `similarity` - The camera's `S_cw`.
/// * `rows` - The points' rows, in range.
/// * `options` - `tracking.view_cos_limit`.
///
/// # Returns
///
/// Those in view, in order.
///
/// # Errors
///
/// The level prediction's, which checked rows never meet.
fn in_view(
    m: &Map,
    camera: &Camera,
    similarity: &Matrix4<f64>,
    rows: &[usize],
    options: &Options,
) -> Result<InView> {
    let pose = rigid_of(similarity);
    let centre = camera_centre(&pose);
    let mut out = InView {
        rows: Vec::new(),
        pixels: Vec::new(),
        levels: Vec::new(),
    };
    let mut distances = Vec::new();
    for &p in rows {
        let local = to_camera(&pose, &m.pt_xyz[p]);
        if !(local.z > 0.0) {
            continue;
        }
        let pixel = camera.project(&local);
        if !camera.inside(&pixel) {
            continue;
        }
        let offset = m.pt_xyz[p] - centre;
        let distance = offset.norm();
        let matchable = distance >= m.pt_min_distance[p]
            && distance <= m.pt_max_distance[p]
            && distance > 0.0
            && viewing_cos(m, p, &offset, distance) >= options.tracking.view_cos_limit;
        if !matchable {
            continue;
        }
        out.rows.push(p);
        out.pixels.push(pixel);
        distances.push(distance);
    }
    out.levels = predict_level(m, &out.rows, &distances)?;
    Ok(out)
}

/// A keyframe's keypoint rows within a square window, on a level or the one below
/// (`slam_it.edge.loops._candidates_near`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `pixel` - The window's centre.
/// * `radius` - Its half-width.
/// * `level` - The level.
///
/// # Returns
///
/// The rows, ascending.
#[allow(clippy::cast_possible_wrap, reason = "levels are at most MAX_LEVELS")]
fn candidates_near(
    m: &Map,
    k: usize,
    pixel: &Vector2<f64>,
    radius: f64,
    level: usize,
) -> Vec<usize> {
    let (low, high) = (level as i64 - 1, level as i64);
    (m.kf_start[k]..m.kf_start[k + 1])
        .filter(|&r| {
            let [x, y] = m.kp_xy[r];
            let at = m.kp_level[r] as i64;
            (x - pixel.x).abs() < radius && (y - pixel.y).abs() < radius && at >= low && at <= high
        })
        .collect()
}

/// The nearest of some keypoints to a point by Hamming distance (ties: the first).
///
/// # Arguments
///
/// * `descriptor` - The point's descriptor.
/// * `m` - The keypoints' map.
/// * `near` - The keypoints' rows, at least one.
///
/// # Returns
///
/// Its index in `near`, and its distance.
fn nearest(descriptor: &Descriptor, m: &Map, near: &[usize]) -> (usize, i64) {
    let mut best = (0, i64::MAX);
    for (i, &r) in near.iter().enumerate() {
        let d = orb::distance(descriptor, &m.kp_descriptor[r]);
        if d < best.1 {
            best = (i, d);
        }
    }
    best
}

/// A search through a similarity: its window and its largest Hamming distance.
#[derive(Debug, Clone, Copy)]
pub struct Search {
    /// The window's half-width, in pixels at level 0.
    pub window: f64,
    /// The largest Hamming distance a match may have.
    pub distance: i64,
}

/// The map a place is looked for in, and its database (`slam_it.edge.loops._Searched`): the new
/// keyframe's own for a loop, whose links a group must not hold; another map's for a merge.
#[derive(Clone, Copy)]
struct Target<'a> {
    /// The map searched.
    map: &'a Map,
    /// Its keyframe database.
    database: &'a Database,
    /// Whether it is the new keyframe's own map.
    own: bool,
}

/// Matches points of `source` to keyframe `k` of `m` through a similarity
/// (`slam_it.edge.loops._search_by_similarity`), the rows in range.
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `similarity` - Its `S_cw`, in `source`'s world.
/// * `source` - The points' map (`m`, for a loop).
/// * `points` - The points' rows in it, in the order searched.
/// * `search` - The window and the distance.
/// * `options` - `tracking.view_cos_limit`.
///
/// # Returns
///
/// The id, in `source`, of the point each of the keyframe's keypoints matched, -1 where none.
///
/// # Errors
///
/// The level prediction's, which rows in range never meet.
#[allow(
    clippy::cast_precision_loss,
    clippy::too_many_arguments,
    reason = "levels are at most MAX_LEVELS; the keyframe with its map, the points with theirs"
)]
fn search_in(
    m: &Map,
    camera: &Camera,
    k: usize,
    similarity: &Matrix4<f64>,
    source: &Map,
    points: &[usize],
    search: Search,
    options: &Options,
) -> Result<Vec<i64>> {
    let start = m.kf_start[k];
    let mut out = vec![-1; m.kf_start[k + 1] - start];
    let seen = in_view(source, camera, similarity, points, options)?;
    for ((&p, pixel), &level) in seen.rows.iter().zip(&seen.pixels).zip(&seen.levels) {
        let radius = search.window * m.scale_factor.powf(level as f64);
        let near: Vec<usize> = candidates_near(m, k, pixel, radius, level)
            .into_iter()
            .filter(|&r| out[r - start] < 0)
            .collect();
        if near.is_empty() {
            continue;
        }
        let (best, distance) = nearest(&source.pt_descriptor[p], m, &near);
        if distance <= search.distance {
            out[near[best] - start] = source.pt_id[p];
        }
    }
    Ok(out)
}

/// Matches points to a keyframe's keypoints through a similarity
/// (`slam_it.edge.loops.search_by_similarity`, ORB-SLAM's `SearchByProjection`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `similarity` - Its `S_cw`.
/// * `points` - The points' rows, in the order searched.
/// * `search` - The window and the distance.
/// * `options` - `tracking.view_cos_limit`.
///
/// # Returns
///
/// The id of the point each of the keyframe's keypoints matched, -1 where none.
///
/// # Errors
///
/// `InvalidInput` for a keyframe or point row out of range.
pub fn search_by_similarity(
    m: &Map,
    camera: &Camera,
    k: usize,
    similarity: &Matrix4<f64>,
    points: &[usize],
    search: Search,
    options: &Options,
) -> Result<Vec<i64>> {
    check_keyframe(m, k, "k")?;
    check_points(m, points)?;
    search_in(m, camera, k, similarity, m, points, search, options)
}

/// One keyframe's side of a match by word: its keypoint rows, those observing a point, and
/// those keypoints' nodes.
type Side = (std::ops::Range<usize>, Vec<usize>, Vec<usize>);

/// One keyframe's side of a match by word, read from its map and database.
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `db` - Its database.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// Its keypoint rows, those observing a point, and their nodes.
///
/// # Errors
///
/// `InvalidInput` for a keyframe the database does not hold with one node per keypoint.
fn side(m: &Map, db: &Database, k: usize) -> Result<Side> {
    let rows = m.kf_start[k]..m.kf_start[k + 1];
    let Some(words) = db.keyframes.get(&m.kf_id[k]) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("database: keyframe {} is not in it", m.kf_id[k]),
        );
    };
    if words.nodes.len() != rows.len() {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "database: keyframe {}'s features are not the map's",
                m.kf_id[k]
            ),
        );
    }
    let own: Vec<usize> = rows.clone().filter(|&r| m.kp_point[r] >= 0).collect();
    let nodes = own.iter().map(|&r| words.nodes[r - rows.start]).collect();
    Ok((rows, own, nodes))
}

/// Matches keyframe `k1` of `m` to `k2` of the searched map by their points' words
/// (`slam_it.edge.loops._match_by_word`), the rows in range.
///
/// # Arguments
///
/// * `m` - The first keyframe's map.
/// * `db` - Its database.
/// * `k1` - The first keyframe's row.
/// * `target` - The second's map, with its database (`m`'s own, for a loop).
/// * `k2` - The second keyframe's row.
/// * `options` - `matching.th_low` and `loops.ratio`.
///
/// # Returns
///
/// For each keypoint of `k1`, the id of the point of `k2` it matched, -1 where none.
///
/// # Errors
///
/// `InvalidInput` for a keyframe a database does not hold with one node per keypoint.
fn match_words(
    m: &Map,
    db: &Database,
    k1: usize,
    target: Target<'_>,
    k2: usize,
    options: &Options,
) -> Result<Vec<i64>> {
    let other = target.map;
    let (rows1, own1, nodes1) = side(m, db, k1)?;
    let (_, own2, nodes2) = side(other, target.database, k2)?;
    let descriptors = |map: &Map, own: &[usize]| {
        own.iter()
            .map(|&r| map.kp_descriptor[r])
            .collect::<Vec<_>>()
    };
    let angles =
        |map: &Map, own: &[usize]| own.iter().map(|&r| map.kp_angle[r]).collect::<Vec<_>>();
    let found = matching::search_by_bow(
        &descriptors(m, &own1),
        &nodes1,
        &descriptors(other, &own2),
        &nodes2,
        &angles(m, &own1),
        &angles(other, &own2),
        &matching::Options {
            max_distance: options.matching.th_low,
            ratio: options.loops.ratio,
            check_orientation: true,
        },
    )?;
    let mut out = vec![-1; rows1.len()];
    for pair in &found {
        out[own1[pair.first] - rows1.start] = other.kp_point[own2[pair.second]];
    }
    Ok(out)
}

/// Matches two keyframes' points by word (`slam_it.edge.loops.match_by_word`, ORB-SLAM's
/// keyframe-to-keyframe `SearchByBoW`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `db` - Its keyframe database, holding both keyframes.
/// * `k1`, `k2` - The two keyframes' rows.
/// * `options` - `matching.th_low` and `loops.ratio`.
///
/// # Returns
///
/// For each keypoint of `k1`, the id of the point of `k2` it matched, -1 where none.
///
/// # Errors
///
/// `InvalidInput` for a row out of range, or a keyframe the database does not hold with one
/// node per keypoint.
pub fn match_by_word(
    m: &Map,
    db: &Database,
    k1: usize,
    k2: usize,
    options: &Options,
) -> Result<Vec<i64>> {
    check_keyframe(m, k1, "k1")?;
    check_keyframe(m, k2, "k2")?;
    let own = Target {
        map: m,
        database: db,
        own: true,
    };
    match_words(m, db, k1, own, k2, options)
}

/// The points some keyframes observe, each once, in the keyframes' order
/// (`slam_it.edge.loops._keyframe_points`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `keyframes` - The keyframes' rows.
///
/// # Returns
///
/// The points' rows.
fn keyframe_points(m: &Map, keyframes: &[usize]) -> Vec<usize> {
    let mut taken = BTreeSet::new();
    let mut out = Vec::new();
    for &k in keyframes {
        for &id in &m.kp_point[m.kf_start[k]..m.kf_start[k + 1]] {
            if id >= 0
                && taken.insert(id)
                && let Some(p) = m.point_row(id)
            {
                out.push(p);
            }
        }
    }
    out
}

/// A keyframe's links' rows, most shared first (`GetVectorCovisibleKeyFrames`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `options` - `covisibility.min_points`.
///
/// # Returns
///
/// The rows.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn links(m: &Map, k: usize, options: &Options) -> Result<Vec<usize>> {
    Ok(covisibility::links(m, k, options)?
        .into_iter()
        .map(|link| link.row)
        .collect())
}

/// A keyframe's best `loops.neighbours` links' rows (`slam_it.edge.loops._best_links`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `options` - The settings.
///
/// # Returns
///
/// The rows.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn best_links(m: &Map, k: usize, options: &Options) -> Result<Vec<usize>> {
    let mut out = links(m, k, options)?;
    out.truncate(options.loops.neighbours);
    Ok(out)
}

/// A keyframe's links' rows, all of them (`GetConnectedKeyFrames`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `options` - The settings.
///
/// # Returns
///
/// The rows.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn linked(m: &Map, k: usize, options: &Options) -> Result<BTreeSet<usize>> {
    Ok(links(m, k, options)?.into_iter().collect())
}

/// The points a matched keyframe's group sees (`slam_it.edge.loops.group_points`, ORB-SLAM's
/// `FindMatchesByProjection`): the keyframe's best `loops.neighbours` links and itself; with
/// fewer links than that, each link's own best, at most `loops.neighbours` of each, but those
/// already in and those of `linked`. Their points, each once, in that order.
///
/// # Arguments
///
/// * `m` - The map.
/// * `matched` - The matched keyframe's row.
/// * `linked` - The rows of the searching keyframe's links, never added by widening.
/// * `options` - `loops.neighbours` and `covisibility.min_points`.
///
/// # Returns
///
/// The points' rows.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
pub fn group_points(
    m: &Map,
    matched: usize,
    linked: &BTreeSet<usize>,
    options: &Options,
) -> Result<Vec<usize>> {
    check_keyframe(m, matched, "matched")?;
    let first = best_links(m, matched, options)?;
    let mut group = first.clone();
    group.push(matched);
    if first.len() < options.loops.neighbours {
        let mut inside: BTreeSet<usize> = group.iter().copied().collect();
        for &row in &first {
            let mut added = 0;
            for other in best_links(m, row, options)? {
                if added == options.loops.neighbours {
                    break;
                }
                if !inside.contains(&other) && !linked.contains(&other) {
                    inside.insert(other);
                    group.push(other);
                    added += 1;
                }
            }
        }
    }
    Ok(keyframe_points(m, &group))
}

// --- the similarity ------------------------------------------------------------------------

/// The most samples the loop's RANSAC draws (`slam_it.edge.loops.ransac_budget`, ORB-SLAM's
/// `SetRansacParameters`): `ceil(log(1 - p) / log(1 - e³))` for `e = loops.min_inliers /
/// matches`, at most `loops.ransac_iterations` and at least 1; 1 when every match must be an
/// inlier.
///
/// # Arguments
///
/// * `matches` - The matches.
/// * `options` - The `loops` settings.
///
/// # Returns
///
/// The samples.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "match counts are far below 2^52; the ceiling is small and positive"
)]
pub fn ransac_budget(matches: usize, options: &Options) -> usize {
    let settings = &options.loops;
    if settings.min_inliers >= matches {
        return 1;
    }
    let bound = ransac::required_iterations(
        settings.min_inliers as f64 / matches as f64,
        sim3::SAMPLE_SIZE,
        settings.confidence,
    );
    (bound.ceil() as usize)
        .min(settings.ransac_iterations)
        .max(1)
}

/// A similarity refined against a keyframe's matches (`slam_it.edge.loops.Refined`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Refined {
    /// The refined `S_12`.
    pub similarity: Matrix4<f64>,
    /// The matches whose errors stayed within `loops.huber_chi2` in both images.
    pub inliers: usize,
}

/// The matches a refinement weighs, row for row.
struct RefinementMatches {
    /// Each match's point in camera 1's frame.
    points1: Vec<Vector3<f64>>,
    /// Each match's point in camera 2's frame.
    points2: Vec<Vector3<f64>>,
    /// Where image 1 observes each.
    pixels1: Vec<Vector2<f64>>,
    /// Where image 2 observes each, NaN where it does not.
    pixels2: Vec<Vector2<f64>>,
    /// Each observation's information, image 1 then image 2.
    information: Vec<[f64; 2]>,
}

impl RefinementMatches {
    /// The matches a mask keeps.
    ///
    /// # Arguments
    ///
    /// * `keep` - One flag per match.
    ///
    /// # Returns
    ///
    /// Those matches.
    fn kept(&self, keep: &[bool]) -> Self {
        let pick = |values: &[Vector3<f64>]| -> Vec<Vector3<f64>> {
            values
                .iter()
                .zip(keep)
                .filter(|(_, k)| **k)
                .map(|(v, _)| *v)
                .collect()
        };
        let pick2 = |values: &[Vector2<f64>]| -> Vec<Vector2<f64>> {
            values
                .iter()
                .zip(keep)
                .filter(|(_, k)| **k)
                .map(|(v, _)| *v)
                .collect()
        };
        Self {
            points1: pick(&self.points1),
            points2: pick(&self.points2),
            pixels1: pick2(&self.pixels1),
            pixels2: pick2(&self.pixels2),
            information: self
                .information
                .iter()
                .zip(keep)
                .filter(|(_, k)| **k)
                .map(|(v, _)| *v)
                .collect(),
        }
    }

    /// Refines a similarity against the matches.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting `S_12`.
    /// * `camera` - The camera.
    /// * `fixed_scale` - Refine a rigid motion.
    /// * `huber` - The Huber loss's threshold, or none.
    /// * `iterations` - The iterations.
    ///
    /// # Returns
    ///
    /// The refinement.
    ///
    /// # Errors
    ///
    /// The solve's.
    fn refine(
        &self,
        start: &Matrix4<f64>,
        camera: &Camera,
        fixed_scale: bool,
        huber: Option<f64>,
        iterations: usize,
    ) -> Result<graphs::SimilarityRefinement> {
        graphs::refine_similarity(
            start,
            &graphs::Matches {
                points1: &self.points1,
                points2: &self.points2,
                pixels1: &self.pixels1,
                pixels2: &self.pixels2,
                information: &self.information,
            },
            camera.model,
            &camera.intrinsics,
            fixed_scale,
            huber,
            &graphs::Options {
                max_iterations: iterations,
                ..graphs::Options::default()
            },
        )
    }
}

/// Which matches a refinement kept within `loops.huber_chi2` in both images.
///
/// # Arguments
///
/// * `found` - The refinement.
/// * `options` - `loops.huber_chi2`.
///
/// # Returns
///
/// One flag per match.
fn within(found: &graphs::SimilarityRefinement, options: &Options) -> Vec<bool> {
    let bound = options.loops.huber_chi2;
    found
        .chi2
        .iter()
        .map(|[one, two]| *one <= bound && !(*two > bound))
        .collect()
}

/// Refines the similarity between two keyframes against their matches
/// (`slam_it.edge.loops.refine`, ORB-SLAM's `OptimizeSim3`): a first round with a Huber loss,
/// the matches past `loops.huber_chi2` dropped, a second with no robust loss.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k1`, `k2` - The two keyframes' rows.
/// * `similarity` - The starting `S_12`, from camera 2's frame to camera 1's.
/// * `matched` - For each keypoint of `k1`, the id of the point it matched, -1 for none.
/// * `fixed_scale` - Refine a rigid motion.
/// * `options` - The `loops` refinement's settings.
///
/// # Returns
///
/// The similarity and its inliers; `None` when fewer than `loops.min_kept` matches survive the
/// first round, or no match can be used.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or matches not one per keypoint of `k1`; the solve's.
#[allow(
    clippy::too_many_arguments,
    reason = "the map, the two keyframes, the start, the matches, the settings"
)]
pub fn refine(
    m: &Map,
    camera: &Camera,
    k1: usize,
    k2: usize,
    similarity: &Matrix4<f64>,
    matched: &[i64],
    fixed_scale: bool,
    options: &Options,
) -> Result<Option<Refined>> {
    check_keyframe(m, k1, "k1")?;
    check_keyframe(m, k2, "k2")?;
    check_keypoint_ids(m, k1, matched, "matched")?;
    refined(
        m,
        camera,
        k1,
        m,
        k2,
        similarity,
        matched,
        fixed_scale,
        options,
    )
}

/// Refines the similarity between keyframe `k1` of `m` and `k2` of `other`
/// (`slam_it.edge.loops._refine`), the rows and matches checked.
///
/// # Arguments
///
/// * `m` - The first keyframe's map.
/// * `camera` - The camera.
/// * `k1` - The first keyframe's row.
/// * `other` - The second's map (`m`, for a loop).
/// * `k2` - The second's row.
/// * `similarity` - The starting `S_12`.
/// * `matched` - For each keypoint of `k1`, the id of the point of `other` it matched, or -1.
/// * `fixed_scale` - Refine a rigid motion.
/// * `options` - The `loops` refinement's settings.
///
/// # Returns
///
/// The similarity and its inliers, or `None`.
///
/// # Errors
///
/// The solve's.
#[allow(
    clippy::too_many_arguments,
    reason = "the first keyframe with its map, the second with its, the start, the matches"
)]
fn refined(
    m: &Map,
    camera: &Camera,
    k1: usize,
    other: &Map,
    k2: usize,
    similarity: &Matrix4<f64>,
    matched: &[i64],
    fixed_scale: bool,
    options: &Options,
) -> Result<Option<Refined>> {
    let settings = &options.loops;
    let mut rows = Vec::new();
    let mut partners = Vec::new();
    let mut weighed = RefinementMatches {
        points1: Vec::new(),
        points2: Vec::new(),
        pixels1: Vec::new(),
        pixels2: Vec::new(),
        information: Vec::new(),
    };
    for (i, row) in (m.kf_start[k1]..m.kf_start[k1 + 1]).enumerate() {
        if matched[i] < 0 || m.kp_point[row] < 0 {
            continue;
        }
        let (Some(own), Some(partner)) =
            (m.point_row(m.kp_point[row]), other.point_row(matched[i]))
        else {
            continue;
        };
        let point1 = to_camera(&m.kf_pose[k1], &m.pt_xyz[own]);
        let point2 = to_camera(&other.kf_pose[k2], &other.pt_xyz[partner]);
        if !(point1.z > MIN_DEPTH && point2.z > MIN_DEPTH) {
            continue;
        }
        rows.push(row);
        partners.push(partner);
        weighed.points1.push(point1);
        weighed.points2.push(point2);
    }
    if rows.is_empty() {
        return Ok(None);
    }
    let rows2 = other.kf_start[k2]..other.kf_start[k2 + 1];
    for (&row, &partner) in rows.iter().zip(&partners) {
        let id = other.pt_id[partner];
        let (pixel, level) = rows2.clone().find(|&r| other.kp_point[r] == id).map_or(
            (Vector2::repeat(f64::NAN), 0),
            |r| {
                (
                    Vector2::new(other.kp_xy[r][0], other.kp_xy[r][1]),
                    other.kp_level[r],
                )
            },
        );
        weighed
            .pixels1
            .push(Vector2::new(m.kp_xy[row][0], m.kp_xy[row][1]));
        weighed.pixels2.push(pixel);
        weighed
            .information
            .push([1.0 / sigma2(m, m.kp_level[row]), 1.0 / sigma2(other, level)]);
    }
    let first = weighed.refine(
        similarity,
        camera,
        fixed_scale,
        Some(settings.huber_chi2.sqrt()),
        settings.refine_iterations,
    )?;
    let good = within(&first, options);
    let kept = good.iter().filter(|&&g| g).count();
    if kept < settings.min_kept {
        return Ok(None);
    }
    let more = if good.iter().all(|&g| g) {
        settings.refine_iterations
    } else {
        settings.refine_more
    };
    let second = weighed
        .kept(&good)
        .refine(&first.similarity, camera, fixed_scale, None, more)?;
    let inliers = within(&second, options).iter().filter(|&&g| g).count();
    Ok(Some(Refined {
        similarity: second.similarity,
        inliers,
    }))
}

// --- detection -----------------------------------------------------------------------------

/// What detection made of a keyframe, with the codes it crosses as
/// (`slam_it.edge.loops.Outcome`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// The map held too few keyframes to search (`loops.min_keyframes`).
    Skipped = 0,
    /// No loop: none was being confirmed, and no candidate passed.
    None = 1,
    /// A candidate passed, with too few confirmations to close: carried to what follows.
    Started = 2,
    /// The loop being confirmed was confirmed again, not yet enough to close.
    Confirmed = 3,
    /// A loop closed on this keyframe.
    Closed = 4,
    /// The loop being confirmed went unconfirmed `loops.misses` keyframes in a row.
    Dropped = 5,
    /// A loop closed on this keyframe was dropped for a merge found on it (`merge_first`).
    Merged = 6,
}

impl Outcome {
    /// Every outcome, in code order.
    pub const ALL: [Self; 7] = [
        Self::Skipped,
        Self::None,
        Self::Started,
        Self::Confirmed,
        Self::Closed,
        Self::Dropped,
        Self::Merged,
    ];

    /// The outcome's name, as `slam_it.edge.loops.Outcome` spells it.
    ///
    /// # Returns
    ///
    /// A static string.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Skipped => "skipped",
            Self::None => "none",
            Self::Started => "started",
            Self::Confirmed => "confirmed",
            Self::Closed => "closed",
            Self::Dropped => "dropped",
            Self::Merged => "merged",
        }
    }
}

/// A loop being confirmed, carried from keyframe to keyframe (`slam_it.edge.loops.Pending`,
/// ORB-SLAM3's loop state).
#[derive(Debug, Clone, PartialEq)]
pub struct Pending {
    /// The id of the keyframe it matched.
    pub matched: i64,
    /// The id of the last keyframe that confirmed it.
    pub last: i64,
    /// That keyframe's `S_cw` as the loop places it.
    pub similarity: Matrix4<f64>,
    /// Its confirmations so far.
    pub confirmations: usize,
    /// The keyframes in a row that failed to confirm it since.
    pub misses: usize,
    /// The ids of the matched keyframe's group's points.
    pub points: Vec<i64>,
    /// For each keypoint of the last keyframe, the id of the group's point it matched, -1 for
    /// none.
    pub matched_points: Vec<i64>,
}

impl Pending {
    /// The keyframes it holds, which culling spares: the matched one and the last, ascending.
    ///
    /// # Returns
    ///
    /// Their ids.
    #[must_use]
    pub fn held(&self) -> Vec<i64> {
        let mut out = vec![self.matched, self.last];
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// A loop closed, ready to correct (`slam_it.edge.loops.Loop`).
#[derive(Debug, Clone, PartialEq)]
pub struct Loop {
    /// The id of the keyframe it closed on.
    pub current: i64,
    /// The id of the keyframe it matched.
    pub matched: i64,
    /// The current keyframe's `S_cw` as the loop places it.
    pub similarity: Matrix4<f64>,
    /// The ids of the matched keyframe's group's points.
    pub points: Vec<i64>,
    /// For each keypoint of the current keyframe, the id of the group's point it matched, -1
    /// for none.
    pub matched_points: Vec<i64>,
}

/// How far one candidate got (`slam_it.edge.loops.CandidateSummary`); each count `None` where
/// it stopped first.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CandidateSummary {
    /// The candidate's id.
    pub keyframe: i64,
    /// Its group's accumulated score.
    pub score: f64,
    /// Whether its group holds a keyframe linked to the new one, so it was not tried.
    pub near: bool,
    /// The new keyframe's points matched by word to the group's.
    pub matches: Option<usize>,
    /// The id of the group's keyframe that matched most.
    pub best: Option<i64>,
    /// The RANSAC's inliers, if it found a similarity.
    pub inliers: Option<usize>,
    /// The first search's matches through it.
    pub projected: Option<usize>,
    /// The refinement's inliers.
    pub refined: Option<usize>,
    /// The second search's matches through the refined similarity.
    pub reprojected: Option<usize>,
    /// The new keyframe's links that confirmed it.
    pub confirmations: Option<usize>,
}

/// How a loop being confirmed fared on a later keyframe
/// (`slam_it.edge.loops.ConfirmationSummary`); each count `None` where it stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ConfirmationSummary {
    /// The id of the keyframe the loop matched.
    pub matched: i64,
    /// The search's matches through the similarity carried to the keyframe.
    pub projected: usize,
    /// The refinement's inliers.
    pub refined: Option<usize>,
    /// The search's matches through the refined similarity.
    pub reprojected: Option<usize>,
    /// Whether it confirmed the loop.
    pub confirmed: bool,
}

/// How detection went on a keyframe (`slam_it.edge.loops.DetectionSummary`).
#[derive(Debug, Clone, PartialEq)]
pub struct DetectionSummary {
    /// What it made of the keyframe.
    pub outcome: Outcome,
    /// The loop being confirmed, carried to it; `None` if there was none.
    pub confirmation: Option<ConfirmationSummary>,
    /// The keyframes outside its links that share a word with it.
    pub sharing: usize,
    /// Those scored.
    pub scored: usize,
    /// The candidates tried, in order.
    pub candidates: Vec<CandidateSummary>,
    /// The loop's confirmations after this keyframe (0 if none is being confirmed).
    pub confirmations: usize,
}

impl DetectionSummary {
    /// A summary of an outcome alone.
    ///
    /// # Arguments
    ///
    /// * `outcome` - The outcome.
    ///
    /// # Returns
    ///
    /// The summary, nothing searched.
    #[must_use]
    pub const fn of(outcome: Outcome) -> Self {
        Self {
            outcome,
            confirmation: None,
            sharing: 0,
            scored: 0,
            candidates: Vec::new(),
            confirmations: 0,
        }
    }
}

/// Detection's result (`slam_it.edge.loops.Detection`).
#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    /// The loop being confirmed, carried to the keyframes that follow; `None` if none is.
    pub pending: Option<Pending>,
    /// The loop closed on this keyframe, to correct; `None` if none closed.
    pub closed: Option<Loop>,
    /// How it went.
    pub summary: DetectionSummary,
}

/// Searches a matched keyframe's group's points in a keyframe through a similarity
/// (`slam_it.edge.loops._confirm_by_projection`, ORB-SLAM's `FindMatchesByProjection`): the
/// group in the map searched, the keyframe's links left out in its own.
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `matched` - The matched keyframe's row, in the map searched.
/// * `similarity` - The keyframe's `S_cw`, in the searched map's world.
/// * `target` - The map searched.
/// * `options` - The settings.
///
/// # Returns
///
/// The group's points' ids, and each of the keyframe's keypoints' match.
///
/// # Errors
///
/// A search's.
fn confirm_by_projection(
    m: &Map,
    camera: &Camera,
    k: usize,
    matched: usize,
    similarity: &Matrix4<f64>,
    target: Target<'_>,
    options: &Options,
) -> Result<(Vec<i64>, Vec<i64>)> {
    let linked = if target.own {
        linked(m, k, options)?
    } else {
        BTreeSet::new()
    };
    let other = target.map;
    let points = group_points(other, matched, &linked, options)?;
    let search = Search {
        window: options.loops.confirm_window,
        distance: options.loops.confirm_distance,
    };
    let found = search_in(m, camera, k, similarity, other, &points, search, options)?;
    Ok((points.iter().map(|&p| other.pt_id[p]).collect(), found))
}

/// Tries to confirm the loop being confirmed on a later keyframe
/// (`slam_it.edge.loops._carry`, `DetectAndReffineSim3FromLastKF`).
///
/// # Arguments
///
/// * `m` - The keyframe's map, holding the loop's last keyframe.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `pending` - The loop.
/// * `target` - The map holding its matched keyframe.
/// * `fixed_scale` - Whether the similarity is rigid.
/// * `options` - The settings.
///
/// # Returns
///
/// The loop as it stands after the keyframe, `None` once dropped, and how it went.
///
/// # Errors
///
/// `InvalidInput` for a keyframe of the loop its map does not hold; a search's or the
/// refinement's.
fn carry(
    m: &Map,
    camera: &Camera,
    k: usize,
    pending: &Pending,
    target: Target<'_>,
    fixed_scale: bool,
    options: &Options,
) -> Result<(Option<Pending>, ConfirmationSummary)> {
    let settings = &options.loops;
    let other = target.map;
    let matched = keyframe_row(other, pending.matched)?;
    let last = keyframe_row(m, pending.last)?;
    let moved = relative(&m.kf_pose[k], &m.kf_pose[last]) * pending.similarity;
    let (_, found) = confirm_by_projection(m, camera, k, matched, &moved, target, options)?;
    let mut summary = ConfirmationSummary {
        matched: pending.matched,
        projected: matched_count(&found),
        ..ConfirmationSummary::default()
    };
    let mut confirmed = None;
    if summary.projected >= settings.confirm_matches {
        let start = moved * sim3::inverse(&other.kf_pose[matched]);
        let refinement = refined(
            m,
            camera,
            k,
            other,
            matched,
            &start,
            &found,
            fixed_scale,
            options,
        )?;
        summary.refined = refinement.map(|r| r.inliers);
        if let Some(refinement) = refinement
            && refinement.inliers > settings.temporal_refined
        {
            let placed = refinement.similarity * other.kf_pose[matched];
            let (points, again) =
                confirm_by_projection(m, camera, k, matched, &placed, target, options)?;
            summary.reprojected = Some(matched_count(&again));
            if matched_count(&again) >= settings.temporal_matches {
                confirmed = Some((placed, points, again));
            }
        }
    }
    let Some((placed, points, again)) = confirmed else {
        let misses = pending.misses + 1;
        let left = (misses < settings.misses).then(|| Pending {
            misses,
            ..pending.clone()
        });
        return Ok((left, summary));
    };
    summary.confirmed = true;
    Ok((
        Some(Pending {
            matched: pending.matched,
            last: m.kf_id[k],
            similarity: placed,
            confirmations: pending.confirmations + 1,
            misses: 0,
            points,
            matched_points: again,
        }),
        summary,
    ))
}

/// A candidate that passed every test: where it places the loop and what it matched.
struct Found {
    /// The row of the group's keyframe that matched most.
    best: usize,
    /// The keyframe's `S_cw` as the loop places it.
    similarity: Matrix4<f64>,
    /// The group's points' ids.
    points: Vec<i64>,
    /// The second search's matches.
    matched_points: Vec<i64>,
    /// How many it matched.
    reprojected: usize,
    /// The keyframe's links that confirmed it.
    confirmations: usize,
}

/// The matches by word of a keyframe with a group, merged (`slam_it.edge.loops._merged_by_word`).
struct ByWord {
    /// Each keypoint's point id, -1 where none.
    points: Vec<i64>,
    /// The row of the keyframe each was matched in, `usize::MAX` where none.
    source: Vec<usize>,
    /// The row of the group's keyframe that matched most (ties: the first).
    best: usize,
}

/// Matches a keyframe by word to each of a group's keyframes, and merges the matches: each
/// keypoint takes its first match in the group's order whose point no earlier keypoint took.
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `db` - Its database.
/// * `k` - The keyframe's row.
/// * `group` - The group's rows, at least one, in the map searched.
/// * `target` - The map searched, with its database.
/// * `options` - The settings.
///
/// # Returns
///
/// The merged matches.
///
/// # Errors
///
/// A matching's.
fn merged_by_word(
    m: &Map,
    db: &Database,
    k: usize,
    group: &[usize],
    target: Target<'_>,
    options: &Options,
) -> Result<ByWord> {
    let size = m.kf_start[k + 1] - m.kf_start[k];
    let mut out = ByWord {
        points: vec![-1; size],
        source: vec![usize::MAX; size],
        best: group[0],
    };
    let mut used = BTreeSet::new();
    let mut most = None;
    for &g in group {
        let found = match_words(m, db, k, target, g, options)?;
        let count = matched_count(&found);
        if most.is_none_or(|most| count > most) {
            out.best = g;
            most = Some(count);
        }
        for (i, &id) in found.iter().enumerate() {
            if id >= 0 && out.points[i] < 0 && !used.contains(&id) {
                out.points[i] = id;
                out.source[i] = g;
                used.insert(id);
            }
        }
    }
    Ok(out)
}

/// Finds the similarity from the best keyframe's frame to the keyframe's by RANSAC
/// (`slam_it.edge.loops._similarity_by_ransac`).
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `merged` - Its merged matches by word.
/// * `other` - The matched points' map (`m`, for a loop).
/// * `fixed_scale` - Whether the similarity is rigid.
/// * `options` - The settings.
///
/// # Returns
///
/// `S_kb` and its inliers; or `None` and the inliers it found (`None` if it did not run) when
/// it falls short of `loops.min_inliers`.
///
/// # Errors
///
/// `InvalidInput` from RANSAC's checks.
fn similarity_by_ransac(
    m: &Map,
    camera: &Camera,
    k: usize,
    merged: &ByWord,
    other: &Map,
    fixed_scale: bool,
    options: &Options,
) -> Result<(Option<Matrix4<f64>>, Option<usize>)> {
    let settings = &options.loops;
    let (mut points1, mut points2) = (Vec::new(), Vec::new());
    let (mut sigma2_1, mut sigma2_2) = (Vec::new(), Vec::new());
    for (i, &id) in merged.points.iter().enumerate() {
        if id < 0 {
            continue;
        }
        let row = m.kf_start[k] + i;
        let (Some(own), Some(partner)) = (m.point_row(m.kp_point[row]), other.point_row(id)) else {
            continue;
        };
        let g = merged.source[i];
        let at = (other.kf_start[g]..other.kf_start[g + 1])
            .find(|&r| other.kp_point[r] == id)
            .map_or(0, |r| other.kp_level[r]);
        let in_current = to_camera(&m.kf_pose[k], &m.pt_xyz[own]);
        let in_best = to_camera(&other.kf_pose[merged.best], &other.pt_xyz[partner]);
        if !(in_current.z > MIN_DEPTH && in_best.z > MIN_DEPTH) {
            continue;
        }
        points1.push(in_current);
        points2.push(in_best);
        sigma2_1.push(sigma2(m, m.kp_level[row]));
        sigma2_2.push(sigma2(other, at));
    }
    let count = points1.len();
    if count < settings.min_inliers.max(sim3::SAMPLE_SIZE) {
        return Ok((None, None));
    }
    let run = ransac::Options {
        threshold: settings.chi2.sqrt(),
        confidence: settings.confidence,
        max_iterations: i64::try_from(ransac_budget(count, options)).unwrap_or(i64::MAX),
        seed: options.seed,
        ..ransac::Options::default()
    };
    let lens = sim3::Lens {
        model: camera.model,
        intrinsics: camera.intrinsics.clone(),
    };
    match sim3::sim3_ransac(
        &points1,
        &points2,
        (&sigma2_1, &sigma2_2),
        &lens,
        fixed_scale,
        &run,
    ) {
        Ok(found) => {
            let inliers = found.summary.inlier_count;
            if inliers < settings.min_inliers {
                return Ok((None, Some(inliers)));
            }
            Ok((Some(found.state.model), Some(inliers)))
        }
        Err(error) if error.kind == ErrorKind::Degenerate => Ok((None, Some(0))),
        Err(error) => Err(error),
    }
}

/// Counts the keyframe's best links that confirm the loop, at most `loops.confirmations`
/// (`slam_it.edge.loops._confirmations`).
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `best` - The matched keyframe's row, in the map searched.
/// * `similarity` - The keyframe's `S_cw`.
/// * `target` - The map searched.
/// * `options` - The settings.
///
/// # Returns
///
/// The confirmations.
///
/// # Errors
///
/// A search's.
fn confirmations(
    m: &Map,
    camera: &Camera,
    k: usize,
    best: usize,
    similarity: &Matrix4<f64>,
    target: Target<'_>,
    options: &Options,
) -> Result<usize> {
    let mut count = 0;
    for j in best_links(m, k, options)? {
        if count == options.loops.confirmations {
            break;
        }
        let moved = relative(&m.kf_pose[j], &m.kf_pose[k]) * similarity;
        let (_, found) = confirm_by_projection(m, camera, j, best, &moved, target, options)?;
        if matched_count(&found) >= options.loops.confirm_matches {
            count += 1;
        }
    }
    Ok(count)
}

/// A candidate's loop placed: the keyframe's `S_cw`, the group's points' rows, and the second
/// search's matches.
type Placed = (Matrix4<f64>, Vec<usize>, Vec<i64>);

/// Places a candidate's loop: search, refine, search again (`slam_it.edge.loops._place`).
///
/// # Arguments
///
/// * `m` - The keyframe's map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `best` - The best keyframe's row, in `other`.
/// * `model` - `S_kb`.
/// * `summary` - The candidate's summary, each count reached written in.
/// * `other` - The map searched (`m`, for a loop).
/// * `fixed_scale` - Whether the similarity is rigid.
/// * `options` - The settings.
///
/// # Returns
///
/// The loop placed, or `None` where a test failed.
///
/// # Errors
///
/// A search's or the refinement's.
#[allow(
    clippy::too_many_arguments,
    reason = "the map, its camera, the two keyframes, the start, the record, the map searched"
)]
fn place(
    m: &Map,
    camera: &Camera,
    k: usize,
    best: usize,
    model: &Matrix4<f64>,
    summary: &mut CandidateSummary,
    other: &Map,
    fixed_scale: bool,
    options: &Options,
) -> Result<Option<Placed>> {
    let settings = &options.loops;
    let mut group = best_links(other, best, options)?;
    group.push(best);
    let points = keyframe_points(other, &group);
    let coarse_search = Search {
        window: settings.coarse_window,
        distance: settings.coarse_distance,
    };
    let start = model * other.kf_pose[best];
    let coarse = search_in(m, camera, k, &start, other, &points, coarse_search, options)?;
    summary.projected = Some(matched_count(&coarse));
    if matched_count(&coarse) < settings.coarse_matches {
        return Ok(None);
    }
    let refinement = refined(
        m,
        camera,
        k,
        other,
        best,
        model,
        &coarse,
        fixed_scale,
        options,
    )?;
    summary.refined = refinement.map(|r| r.inliers);
    let Some(refinement) = refinement.filter(|r| r.inliers >= settings.min_refined) else {
        return Ok(None);
    };
    let placed = refinement.similarity * other.kf_pose[best];
    let fine_search = Search {
        window: settings.fine_window,
        distance: settings.fine_distance,
    };
    let fine = search_in(m, camera, k, &placed, other, &points, fine_search, options)?;
    summary.reprojected = Some(matched_count(&fine));
    if matched_count(&fine) < settings.fine_matches {
        return Ok(None);
    }
    Ok(Some((placed, points, fine)))
}

/// What the search of a keyframe needs besides its candidate: the keyframe's map, its database
/// and camera, the keyframe, its links in its own map (none for a merge's search) and the
/// settings.
struct Searching<'a> {
    /// The keyframe's map.
    m: &'a Map,
    /// Its database.
    db: &'a Database,
    /// The camera.
    camera: &'a Camera,
    /// The keyframe's row.
    k: usize,
    /// Its links' rows in its own map, which a group must not hold; none for another map.
    linked: BTreeSet<usize>,
    /// Whether the similarity is rigid.
    fixed_scale: bool,
    /// The settings.
    options: &'a Options,
}

impl<'a> Searching<'a> {
    /// The search of a keyframe, with no links left out (a loop's search adds its own).
    ///
    /// # Arguments
    ///
    /// * `m` - The keyframe's map.
    /// * `db` - Its database.
    /// * `camera` - The camera.
    /// * `k` - The keyframe's row.
    /// * `mode` - The pipeline's mode: the similarity is rigid but in monocular.
    /// * `options` - The settings.
    ///
    /// # Returns
    ///
    /// The search.
    fn new(
        m: &'a Map,
        db: &'a Database,
        camera: &'a Camera,
        k: usize,
        mode: Mode,
        options: &'a Options,
    ) -> Self {
        Self {
            m,
            db,
            camera,
            k,
            linked: BTreeSet::new(),
            fixed_scale: mode != Mode::Monocular,
            options,
        }
    }

    /// Tries one candidate, a keyframe of the map searched (`slam_it.edge.loops._try_candidate`).
    ///
    /// # Arguments
    ///
    /// * `keyframe` - The candidate's id, in the map searched.
    /// * `score` - Its group's score.
    /// * `target` - The map searched, with its database.
    ///
    /// # Returns
    ///
    /// How far it got, and the loop if it passed.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for a candidate the map does not hold; a stage's.
    fn try_candidate(
        &self,
        keyframe: i64,
        score: f64,
        target: Target<'_>,
    ) -> Result<(CandidateSummary, Option<Found>)> {
        let (m, options) = (self.m, self.options);
        let other = target.map;
        let settings = &options.loops;
        let c = keyframe_row(other, keyframe)?;
        let mut group = vec![c];
        group.extend(best_links(other, c, options)?);
        let mut summary = CandidateSummary {
            keyframe,
            score,
            ..CandidateSummary::default()
        };
        if group.iter().any(|g| self.linked.contains(g)) {
            summary.near = true;
            return Ok((summary, None));
        }
        let merged = merged_by_word(m, self.db, self.k, &group, target, options)?;
        summary.matches = Some(matched_count(&merged.points));
        summary.best = Some(other.kf_id[merged.best]);
        if matched_count(&merged.points) < settings.min_matches {
            return Ok((summary, None));
        }
        let (model, inliers) = similarity_by_ransac(
            m,
            self.camera,
            self.k,
            &merged,
            other,
            self.fixed_scale,
            options,
        )?;
        summary.inliers = inliers;
        let Some(model) = model else {
            return Ok((summary, None));
        };
        let Some((similarity, points, fine)) = place(
            m,
            self.camera,
            self.k,
            merged.best,
            &model,
            &mut summary,
            other,
            self.fixed_scale,
            options,
        )?
        else {
            return Ok((summary, None));
        };
        let confirmed = confirmations(
            m,
            self.camera,
            self.k,
            merged.best,
            &similarity,
            target,
            options,
        )?;
        summary.confirmations = Some(confirmed);
        let found = Found {
            best: merged.best,
            similarity,
            points: points.iter().map(|&p| other.pt_id[p]).collect(),
            reprojected: matched_count(&fine),
            matched_points: fine,
            confirmations: confirmed,
        };
        Ok((summary, Some(found)))
    }

    /// Tries the keyframe's own database's candidates (`slam_it.edge.loops._search`).
    ///
    /// # Returns
    ///
    /// The best that passed, and how the search went.
    ///
    /// # Errors
    ///
    /// The database's or a candidate's.
    fn search(&self) -> Result<RegionSearch> {
        let candidates =
            database::loop_candidates(self.db, self.m, self.m.kf_id[self.k], self.options)?;
        let own = Target {
            map: self.m,
            database: self.db,
            own: true,
        };
        let mut out = RegionSearch {
            best: None,
            summary: searched_summary(candidates.sharing, candidates.scored),
            maps: Vec::new(),
        };
        for (&keyframe, &score) in candidates.keyframes.iter().zip(&candidates.scores) {
            let (summary, found) = self.try_candidate(keyframe, score, own)?;
            out.summary.candidates.push(summary);
            out.keep(0, found);
        }
        Ok(out)
    }

    /// Tries the other maps' candidates (`slam_it.edge.loops._search_merge`), each against its
    /// own map, which holds none of the keyframe's links.
    ///
    /// # Arguments
    ///
    /// * `others` - The other maps.
    ///
    /// # Returns
    ///
    /// The best that passed with its map's place among `others`, and how the search went.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for a keyframe the database does not hold; the search's or a candidate's.
    fn search_merge(&self, others: &[MapRef<'_>]) -> Result<RegionSearch> {
        let keyframe = self.m.kf_id[self.k];
        let Some(words) = self.db.keyframes.get(&keyframe) else {
            return fail(
                ErrorKind::InvalidInput,
                format!("database: keyframe {keyframe} is not in it"),
            );
        };
        let candidates = database::merge_candidates(&words.vector, others, self.options)?;
        let mut out = RegionSearch {
            best: None,
            summary: searched_summary(candidates.sharing, candidates.scored),
            maps: candidates.maps.clone(),
        };
        for ((&number, &keyframe), &score) in candidates
            .maps
            .iter()
            .zip(&candidates.keyframes)
            .zip(&candidates.scores)
        {
            let at = place_of(others, number);
            let other = &others[at];
            let target = Target {
                map: other.map,
                database: other.database,
                own: false,
            };
            let (summary, found) = self.try_candidate(keyframe, score, target)?;
            out.summary.candidates.push(summary);
            out.keep(at, found);
        }
        Ok(out)
    }
}

/// A map's place among maps, by its number.
///
/// # Arguments
///
/// * `maps` - The maps.
/// * `number` - The number.
///
/// # Returns
///
/// The place, or the count when no map has it.
fn place_of(maps: &[MapRef<'_>], number: i64) -> usize {
    maps.iter()
        .position(|map| map.number == number)
        .unwrap_or(maps.len())
}

/// A search's summary before its candidates are tried.
///
/// # Arguments
///
/// * `sharing` - The keyframes sharing a word.
/// * `scored` - Those scored.
///
/// # Returns
///
/// The summary, its outcome `None`.
const fn searched_summary(sharing: usize, scored: usize) -> DetectionSummary {
    DetectionSummary {
        outcome: Outcome::None,
        confirmation: None,
        sharing,
        scored,
        candidates: Vec::new(),
        confirmations: 0,
    }
}

/// A search for a region: the best candidate that passed with its map's place among the maps
/// searched (a loop's own, 0), how the search went, and each candidate's map number (none for a
/// loop).
struct RegionSearch {
    /// The best candidate's map's place, and what it found.
    best: Option<(usize, Found)>,
    /// How the search went.
    summary: DetectionSummary,
    /// Each candidate's map number.
    maps: Vec<i64>,
}

impl RegionSearch {
    /// Keeps a candidate that passed if its second search matched more than the best's.
    ///
    /// # Arguments
    ///
    /// * `at` - Its map's place.
    /// * `found` - What it found, if it passed.
    fn keep(&mut self, at: usize, found: Option<Found>) {
        if let Some(found) = found
            && self
                .best
                .as_ref()
                .is_none_or(|(_, best)| found.reprojected > best.reprojected)
        {
            self.best = Some((at, found));
        }
    }
}

/// Closes a loop with enough confirmations, carries it with fewer, keeps none with none
/// (`slam_it.edge.loops._settle`).
///
/// # Arguments
///
/// * `state` - The loop as the keyframe left it, its last the keyframe.
/// * `summary` - What happened if it is carried.
/// * `options` - `loops.confirmations`.
///
/// # Returns
///
/// The detection.
fn settle(state: Pending, mut summary: DetectionSummary, options: &Options) -> Detection {
    summary.confirmations = state.confirmations;
    if state.confirmations >= options.loops.confirmations {
        summary.outcome = Outcome::Closed;
        return Detection {
            pending: None,
            closed: Some(Loop {
                current: state.last,
                matched: state.matched,
                similarity: state.similarity,
                points: state.points,
                matched_points: state.matched_points,
            }),
            summary,
        };
    }
    if state.confirmations == 0 {
        summary.outcome = Outcome::None;
        summary.confirmations = 0;
        return Detection {
            pending: None,
            closed: None,
            summary,
        };
    }
    Detection {
        pending: Some(state),
        closed: None,
        summary,
    }
}

/// Where detection looks: the region being confirmed and the map holding it, and the maps a
/// new region may be found in.
struct Looking<'a> {
    /// The region being confirmed, if any.
    pending: Option<&'a Pending>,
    /// The map holding its matched keyframe, with its database.
    holding: Target<'a>,
    /// That map's place among `maps`.
    holding_at: usize,
    /// The maps a new region may be found in, by place.
    maps: Vec<&'a Map>,
}

/// A region detected: as a loop's detection holds it, with the place of the map it matched
/// among the maps searched, and each candidate's map number.
struct Detected {
    /// The detection.
    detection: Detection,
    /// The place of the map its region matched.
    at: usize,
    /// Each candidate's map number.
    maps: Vec<i64>,
}

/// Detection's flow, a loop's and a merge's alike (`slam_it.edge.loops._detect` and
/// `_detect_merge`): the region being confirmed carried to the keyframe, and, when it is not
/// confirmed there, the search's candidates tried.
///
/// # Arguments
///
/// * `searching` - The keyframe, its map and the settings.
/// * `looking` - Where to look.
/// * `search` - The search of the database's candidates.
///
/// # Returns
///
/// The region detected.
///
/// # Errors
///
/// A stage's.
fn detect_region(
    searching: &Searching<'_>,
    looking: &Looking<'_>,
    search: impl FnOnce(&Searching<'_>) -> Result<RegionSearch>,
) -> Result<Detected> {
    let (m, k, options) = (searching.m, searching.k, searching.options);
    let mut state = None;
    let mut confirmation = None;
    let mut dropped = false;
    if let Some(pending) = looking.pending {
        let (carried, confirmed) = carry(
            m,
            searching.camera,
            k,
            pending,
            looking.holding,
            searching.fixed_scale,
            options,
        )?;
        if confirmed.confirmed
            && let Some(carried) = carried
        {
            let summary = DetectionSummary {
                confirmation: Some(confirmed),
                ..DetectionSummary::of(Outcome::Confirmed)
            };
            return Ok(Detected {
                detection: settle(carried, summary, options),
                at: looking.holding_at,
                maps: Vec::new(),
            });
        }
        dropped = carried.is_none();
        state = carried;
        confirmation = Some(confirmed);
    }
    let found = search(searching)?;
    let mut summary = found.summary;
    summary.confirmation = confirmation;
    let Some((at, best)) = found.best else {
        summary.outcome = if dropped {
            Outcome::Dropped
        } else {
            Outcome::None
        };
        summary.confirmations = state.as_ref().map_or(0, |p| p.confirmations);
        return Ok(Detected {
            detection: Detection {
                pending: state,
                closed: None,
                summary,
            },
            at: looking.holding_at,
            maps: found.maps,
        });
    };
    summary.outcome = Outcome::Started;
    let started = Pending {
        matched: looking.maps[at].kf_id[best.best],
        last: m.kf_id[k],
        similarity: best.similarity,
        confirmations: best.confirmations,
        misses: 0,
        points: best.points,
        matched_points: best.matched_points,
    };
    Ok(Detected {
        detection: settle(started, summary, options),
        at,
        maps: found.maps,
    })
}

/// Looks for a loop at a newly mapped keyframe (`slam_it.edge.loops.detect`, ORB-SLAM3's
/// `NewDetectCommonRegions`): the loop being confirmed carried to the keyframe, and, when it
/// is not confirmed here, the database's candidates tried.
///
/// # Arguments
///
/// * `m` - The map, holding the keyframe.
/// * `db` - Its keyframe database, holding the keyframe.
/// * `camera` - The camera.
/// * `keyframe` - The new keyframe's id.
/// * `pending` - The loop being confirmed, from the keyframes before.
/// * `mode` - The pipeline's mode: the similarity is rigid but in monocular.
/// * `options` - The settings.
///
/// # Returns
///
/// The loop being confirmed after the keyframe, the loop closed on it, and how it went.
///
/// # Errors
///
/// `InvalidInput` if the keyframe is not in the map or the database; a stage's.
pub fn detect(
    m: &Map,
    db: &Database,
    camera: &Camera,
    keyframe: i64,
    pending: Option<&Pending>,
    mode: Mode,
    options: &Options,
) -> Result<Detection> {
    let k = keyframe_row(m, keyframe)?;
    if m.keyframes() < options.loops.min_keyframes {
        return Ok(Detection {
            pending: pending.cloned(),
            closed: None,
            summary: DetectionSummary {
                confirmations: pending.map_or(0, |p| p.confirmations),
                ..DetectionSummary::of(Outcome::Skipped)
            },
        });
    }
    let looking = Looking {
        pending,
        holding: Target {
            map: m,
            database: db,
            own: true,
        },
        holding_at: 0,
        maps: vec![m],
    };
    let searching = Searching::new(m, db, camera, k, mode, options);
    let detected = detect_region(&searching, &looking, |searching| {
        Searching {
            linked: linked(m, k, options)?,
            ..*searching
        }
        .search()
    })?;
    Ok(detected.detection)
}

/// A merge being confirmed (`slam_it.edge.loops.Merging`).
#[derive(Debug, Clone, PartialEq)]
pub struct Merging {
    /// The number of the map it matched.
    pub map: i64,
    /// The region as a loop being confirmed holds it: its matched keyframe that map's, its last
    /// the active map's.
    pub pending: Pending,
}

/// A merge found, ready to make (`slam_it.edge.loops.Merge`).
#[derive(Debug, Clone, PartialEq)]
pub struct Merge {
    /// The number of the map it matched.
    pub map: i64,
    /// The common region as a closed loop holds it: its similarity the current keyframe's
    /// `S_cw` in the matched map's world.
    pub found: Loop,
}

/// Merge detection's result (`slam_it.edge.loops.MergeDetection`).
#[derive(Debug, Clone, PartialEq)]
pub struct MergeDetection {
    /// The merge being confirmed after it, if any.
    pub pending: Option<Merging>,
    /// The merge found on this keyframe, if any.
    pub merge: Option<Merge>,
    /// How it went, as loop detection's says; each candidate's keyframe an id in its map.
    pub summary: DetectionSummary,
    /// Each candidate's map number, in the summary's order.
    pub maps: Vec<i64>,
}

/// Looks for another map at a newly mapped keyframe (`slam_it.edge.loops.detect_merge`,
/// `NewDetectCommonRegions`' merge branch): none below `loops.min_keyframes`; the merge being
/// confirmed carried through its matched map; else the other maps' candidates, each tried
/// against its own map. A confirmation resets the misses, as a loop's does.
///
/// # Arguments
///
/// * `m` - The active map, holding the keyframe.
/// * `db` - Its keyframe database, holding the keyframe.
/// * `camera` - The camera.
/// * `keyframe` - The new keyframe's id.
/// * `others` - The other maps, each with its database and number.
/// * `pending` - The merge being confirmed, if any.
/// * `mode` - The pipeline's mode: the similarity is rigid but in monocular.
/// * `options` - The settings.
///
/// # Returns
///
/// The merge being confirmed after the keyframe, the merge found on it, and how it went.
///
/// # Errors
///
/// `InvalidInput` for a keyframe the map or a database does not hold, two maps with one
/// number, or a merge being confirmed in a map not given; `NotConverged` for GTSAM's failure
/// refining a candidate's similarity; a stage's.
#[allow(
    clippy::too_many_arguments,
    reason = "the map, its database and camera, the keyframe, the others, the state, as Python's"
)]
pub fn detect_merge(
    m: &Map,
    db: &Database,
    camera: &Camera,
    keyframe: i64,
    others: &[MapRef<'_>],
    pending: Option<&Merging>,
    mode: Mode,
    options: &Options,
) -> Result<MergeDetection> {
    let k = keyframe_row(m, keyframe)?;
    database::check_numbers(others.iter().map(|other| other.number), "others")?;
    let holding_at = pending.map(|p| place_of(others, p.map));
    if let (Some(p), Some(at)) = (pending, holding_at)
        && at == others.len()
    {
        return fail(
            ErrorKind::InvalidInput,
            format!("pending.map: map {} is not among the others", p.map),
        );
    }
    if m.keyframes() < options.loops.min_keyframes {
        return Ok(MergeDetection {
            pending: pending.cloned(),
            merge: None,
            summary: DetectionSummary {
                confirmations: pending.map_or(0, |p| p.pending.confirmations),
                ..DetectionSummary::of(Outcome::Skipped)
            },
            maps: Vec::new(),
        });
    }
    let at = holding_at.unwrap_or(0);
    let looking = Looking {
        pending: pending.map(|p| &p.pending),
        holding: match others.get(at) {
            Some(other) => Target {
                map: other.map,
                database: other.database,
                own: false,
            },
            None => Target {
                map: m,
                database: db,
                own: false,
            },
        },
        holding_at: at,
        maps: others.iter().map(|other| other.map).collect(),
    };
    let searching = Searching::new(m, db, camera, k, mode, options);
    let detected = detect_region(&searching, &looking, |searching| {
        searching.search_merge(others)
    })?;
    let number = |at: usize| others[at].number;
    Ok(MergeDetection {
        pending: detected.detection.pending.map(|pending| Merging {
            map: number(detected.at),
            pending,
        }),
        merge: detected.detection.closed.map(|found| Merge {
            map: number(detected.at),
            found,
        }),
        summary: detected.detection.summary,
        maps: detected.maps,
    })
}

/// Gives a merge found on a keyframe priority over a loop closed on it
/// (`slam_it.edge.loops.merge_first`, ORB-SLAM3's `Run`): the loop forgotten, its outcome
/// `Merged`; a loop still being confirmed kept.
///
/// # Arguments
///
/// * `found` - Loop detection's result on the keyframe.
/// * `merging` - Merge detection's on the same keyframe.
///
/// # Returns
///
/// The loop's detection, its loop and state dropped if a merge was found too.
#[must_use]
pub fn merge_first(found: Detection, merging: &MergeDetection) -> Detection {
    if merging.merge.is_none() || found.closed.is_none() {
        return found;
    }
    Detection {
        pending: None,
        closed: None,
        summary: DetectionSummary {
            outcome: Outcome::Merged,
            ..found.summary
        },
    }
}

// --- correction ----------------------------------------------------------------------------

/// How a loop's correction went (`slam_it.edge.loops.CorrectionSummary`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorrectionSummary {
    /// The id of the keyframe the loop closed on.
    pub current: i64,
    /// The id of the keyframe it matched.
    pub matched: i64,
    /// The current keyframe's `S_cw` as the loop placed it.
    pub similarity: Matrix4<f64>,
    /// The keyframes moved by the loop's similarity: the current one and its links.
    pub corrected: usize,
    /// The points moved with them.
    pub moved: usize,
    /// The current keyframe's points the loop's matched points replaced, or joined.
    pub replaced: usize,
    /// The loop's points fused into the moved keyframes.
    pub fused: usize,
    /// The links new across the loop.
    pub links: usize,
    /// The essential graph's edges.
    pub edges: usize,
    /// Its solve.
    pub essential: graphs::Summary,
    /// Global BA's solve; `None` with `loops.global_ba` off.
    pub global_ba: Option<graphs::Summary>,
}

/// Fuses points into a keyframe through a similarity (`slam_it.edge.loops.fuse_by_similarity`,
/// ORB-SLAM's `Fuse` with `Scw`): each point still in the map that the keyframe does not
/// observe, in view, takes the nearest keypoint within `loops.fuse_window` times its predicted
/// level's scale, on that level or the one below, within `matching.th_low`; a free keypoint
/// observes it, and a keypoint observing another point has that point replaced by it.
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `similarity` - Its `S_cw`.
/// * `point_ids` - The points' ids, in the order fused; one no longer in the map is passed over.
/// * `options` - `loops.fuse_window`, `matching.th_low` and `tracking.view_cos_limit`.
///
/// # Returns
///
/// The points fused.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or a negative id; the map's.
#[allow(clippy::cast_precision_loss, reason = "levels are at most MAX_LEVELS")]
pub fn fuse_by_similarity(
    m: &mut Map,
    camera: &Camera,
    k: usize,
    similarity: &Matrix4<f64>,
    point_ids: &[i64],
    options: &Options,
) -> Result<usize> {
    check_keyframe(m, k, "k")?;
    if point_ids.iter().any(|&id| id < 0) {
        return fail(ErrorKind::InvalidInput, "point_ids: expected point ids");
    }
    let keyframe = m.kf_id[k];
    let mut fused = 0;
    for &id in point_ids {
        let k = keyframe_row(m, keyframe)?;
        let rows = m.kf_start[k]..m.kf_start[k + 1];
        let Some(p) = m.point_row(id) else {
            continue;
        };
        if m.kp_point[rows].contains(&id) {
            continue;
        }
        let seen = in_view(m, camera, similarity, &[p], options)?;
        let (Some(pixel), Some(&level)) = (seen.pixels.first(), seen.levels.first()) else {
            continue;
        };
        let radius = options.loops.fuse_window * m.scale_factor.powf(level as f64);
        let near = candidates_near(m, k, pixel, radius, level);
        if near.is_empty() {
            continue;
        }
        let (best, distance) = nearest(&m.pt_descriptor[p], m, &near);
        if distance > options.matching.th_low {
            continue;
        }
        let held = m.kp_point[near[best]];
        if held >= 0 {
            map::replace_point(m, held, id)?;
        } else {
            map::observe(m, near[best], id)?;
            map::refresh_points(m, &[p])?;
        }
        fused += 1;
    }
    Ok(fused)
}

/// Moves keyframes by their corrected similarities, each point with the first that sees it
/// (`slam_it.edge.loops._move_keyframes`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `moved` - The keyframes' rows, in order.
/// * `corrected` - Each one's corrected `S_iw`, by row.
///
/// # Returns
///
/// Each moved point's id to the row of the keyframe that moved it.
///
/// # Errors
///
/// The refresh's, which an observed point never meets.
fn move_keyframes(
    m: &mut Map,
    moved: &[usize],
    corrected: &BTreeMap<usize, Matrix4<f64>>,
) -> Result<BTreeMap<i64, usize>> {
    let original = m.pt_xyz.clone();
    let mut moved_by = BTreeMap::new();
    for &r in moved {
        let moving = sim3::inverse(&corrected[&r]) * m.kf_pose[r];
        for i in m.kf_start[r]..m.kf_start[r + 1] {
            let id = m.kp_point[i];
            if id < 0 || moved_by.contains_key(&id) {
                continue;
            }
            let Some(p) = m.point_row(id) else {
                continue;
            };
            m.pt_xyz[p] = carried(&moving, &original[p]);
            moved_by.insert(id, r);
        }
    }
    for &r in moved {
        m.kf_pose[r] = rigid_of(&corrected[&r]);
    }
    let rows: Vec<usize> = moved_by.keys().filter_map(|&id| m.point_row(id)).collect();
    map::refresh_points(m, &rows)?;
    Ok(moved_by)
}

/// Replaces the keyframe's points by the loop's it matched, or gives its free keypoints them
/// (`slam_it.edge.loops._replace_matched`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `keyframe` - The keyframe's id.
/// * `matched_points` - For each of its keypoints, the loop's point it matched, or -1.
///
/// # Returns
///
/// The keypoints given a point.
///
/// # Errors
///
/// `InvalidInput` for a keyframe the map does not hold; the map's.
fn replace_matched(m: &mut Map, keyframe: i64, matched_points: &[i64]) -> Result<usize> {
    let mut count = 0;
    for (i, &id) in matched_points.iter().enumerate() {
        let k = keyframe_row(m, keyframe)?;
        let row = m.kf_start[k] + i;
        let Some(p) = (id >= 0).then(|| m.point_row(id)).flatten() else {
            continue;
        };
        if m.kp_point[m.kf_start[k]..m.kf_start[k + 1]].contains(&id) {
            continue; // the keyframe sees it already
        }
        let held = m.kp_point[row];
        if held >= 0 {
            map::replace_point(m, held, id)?;
        } else {
            map::observe(m, row, id)?;
            let p = m.point_row(id).unwrap_or(p);
            map::refresh_points(m, &[p])?;
        }
        count += 1;
    }
    Ok(count)
}

/// The points two keyframes share (`slam_it.edge.loops._weight`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `a`, `b` - The keyframes' rows.
///
/// # Returns
///
/// The count.
fn weight(m: &Map, a: usize, b: usize) -> usize {
    let ids = |k: usize| -> BTreeSet<i64> {
        m.kp_point[m.kf_start[k]..m.kf_start[k + 1]]
            .iter()
            .copied()
            .filter(|&id| id >= 0)
            .collect()
    };
    ids(a).intersection(&ids(b)).count()
}

/// The essential graph's edges, as rows, and each one's measured relative similarity.
struct Graph {
    /// The edges.
    edges: Vec<[usize; 2]>,
    /// Each one's `S_i S_j⁻¹`.
    relatives: Vec<Matrix4<f64>>,
}

/// Collects the essential graph's edges (`slam_it.edge.loops._essential_edges`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `start` - Each keyframe's `S_iw` as the graph starts, by row.
/// * `before` - Each one's pose before the correction, by row.
/// * `loop_links` - Each moved keyframe's (row) links new across the loop.
/// * `ends` - The current and matched keyframes' rows.
/// * `options` - The settings.
///
/// # Returns
///
/// The edges.
///
/// # Errors
///
/// `InvalidInput` for a loop edge naming a keyframe the map does not hold.
fn essential_edges(
    m: &Map,
    start: &[Matrix4<f64>],
    before: &[Matrix4<f64>],
    loop_links: &BTreeMap<usize, BTreeSet<usize>>,
    ends: (usize, usize),
    options: &Options,
) -> Result<Graph> {
    let threshold = options.loops.essential_points;
    let mut graph = Graph {
        edges: Vec::new(),
        relatives: Vec::new(),
    };
    let mut add = |i: usize, j: usize, poses: &[Matrix4<f64>]| {
        graph.edges.push([i, j]);
        graph.relatives.push(poses[i] * sim3::inverse(&poses[j]));
    };
    let mut inserted = BTreeSet::new();
    for (&i, linked) in loop_links {
        for &j in linked {
            if (i, j) != ends && weight(m, i, j) < threshold {
                continue;
            }
            add(i, j, start);
            inserted.insert((i.min(j), i.max(j)));
        }
    }
    for i in 0..m.keyframes() {
        let parent = m.keyframe_row(m.kf_parent[i]);
        if let Some(parent) = parent {
            add(i, parent, before);
        }
        for &[older, newer] in &m.loop_edges {
            if newer == m.kf_id[i] {
                add(i, keyframe_row(m, older)?, before);
            }
        }
        for link in covisibility::links(m, i, options)? {
            let n = link.row;
            if link.weight < threshold
                || Some(n) == parent
                || m.kf_parent[n] == m.kf_id[i]
                || m.kf_id[n] > m.kf_id[i]
                || inserted.contains(&(i.min(n), i.max(n)))
            {
                continue;
            }
            add(i, n, before);
        }
    }
    Ok(graph)
}

/// The essential graph optimised (`slam_it.edge.loops.essential_graph`).
#[derive(Debug, Clone)]
pub struct Essential {
    /// Each keyframe's optimised `S_iw`, by row.
    pub poses: Vec<Matrix4<f64>>,
    /// The graph's edges.
    pub edges: usize,
    /// The solve.
    pub summary: graphs::Summary,
}

/// Optimises the essential graph from the loop's start (`slam_it.edge.loops.essential_graph`,
/// ORB-SLAM's `OptimizeEssentialGraph`): unit information, the `fixed` keyframes held (a loop's
/// matched one, a merge's welding windows), `loops.essential_iterations` of Levenberg-Marquardt
/// from `loops.essential_damping`, on Sim(3) but in stereo and RGB-D.
///
/// # Arguments
///
/// * `m` - The map.
/// * `start` - Each keyframe's `S_iw` the graph starts from, by row.
/// * `before` - Each one's pose before the correction, by row.
/// * `loop_links` - Each moved keyframe's (row) links new across the loop.
/// * `ends` - The current and matched keyframes' rows.
/// * `fixed` - The rows held fixed, at least one.
/// * `fixed_scale` - Optimise SE(3), not Sim(3).
/// * `options` - The settings.
///
/// # Returns
///
/// The poses, the edges and the solve.
///
/// # Errors
///
/// `InvalidInput` for poses that are not one per keyframe; the solve's.
#[allow(
    clippy::too_many_arguments,
    reason = "the map, the correction's state, what is held, the settings, as Python's"
)]
pub fn essential_graph(
    m: &Map,
    start: &[Matrix4<f64>],
    before: &[Matrix4<f64>],
    loop_links: &BTreeMap<usize, BTreeSet<usize>>,
    ends: (usize, usize),
    fixed: &[usize],
    fixed_scale: bool,
    options: &Options,
) -> Result<Essential> {
    if start.len() != m.keyframes() || before.len() != m.keyframes() {
        return fail(
            ErrorKind::InvalidInput,
            "start, before: expected one pose per keyframe",
        );
    }
    check_keyframe(m, ends.0, "current")?;
    check_keyframe(m, ends.1, "matched")?;
    let graph = essential_edges(m, start, before, loop_links, ends, options)?;
    let poses: Vec<Matrix4<f64>> = start.iter().map(sim3::inverse).collect();
    let solved = graphs::optimize_pose_graph(
        &poses,
        &graph.edges,
        &graph.relatives,
        fixed,
        !fixed_scale,
        &graphs::Options {
            max_iterations: options.loops.essential_iterations,
            initial_damping: options.loops.essential_damping,
            ..graphs::Options::default()
        },
    )?;
    Ok(Essential {
        poses: solved.poses.iter().map(sim3::inverse).collect(),
        edges: graph.edges.len(),
        summary: solved.summary,
    })
}

/// Bundle-adjusts cameras and points through the camera (`graphs::bundle_adjust`): global BA's
/// and the welding BA's solve.
///
/// # Arguments
///
/// * `poses` - Each camera's `T_world_cam`.
/// * `points` - Each point.
/// * `observations` - The observations.
/// * `camera` - The camera.
/// * `fixed` - The cameras held.
/// * `iterations` - The solve's iterations.
/// * `stop` - Stops it before its next iteration (global BA in a thread of its own).
///
/// # Returns
///
/// The adjustment.
///
/// # Errors
///
/// The solve's.
fn adjust(
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    observations: &graphs::Observations<'_>,
    camera: &Camera,
    fixed: &[usize],
    iterations: usize,
    stop: &Stop,
) -> Result<graphs::Adjustment> {
    graphs::bundle_adjust(
        poses,
        points,
        observations,
        &graphs::Lens {
            model: camera.model,
            intrinsics: camera.intrinsics.clone(),
            bf: camera.focal() * camera.baseline,
        },
        fixed,
        0.0,
        &graphs::Options {
            max_iterations: iterations,
            stop: stop.clone(),
            ..graphs::Options::default()
        },
    )
}

/// Bundle-adjusts every keyframe and point (`slam_it.edge.loops.global_bundle_adjust`,
/// `GlobalBundleAdjustemnt` as loop closing runs it): every observation, weighed by its
/// level's information, with its right column where it has one; no robust loss; the map's
/// first keyframe (id 0) held fixed; `loops.global_iterations`. Every point is refreshed after.
///
/// # Arguments
///
/// * `m` - The map, changed in place, with its first keyframe.
/// * `camera` - The camera.
/// * `options` - `loops.global_iterations`.
/// * `stop` - Stops it before its next iteration (a global BA in a thread of its own,
///   superseded: ORB-SLAM's `mbStopGBA`).
///
/// # Returns
///
/// The solve's summary.
///
/// # Errors
///
/// `InvalidInput` if the map does not hold its first keyframe; the solve's.
pub fn global_bundle_adjust(
    m: &mut Map,
    camera: &Camera,
    options: &Options,
    stop: &Stop,
) -> Result<graphs::Summary> {
    if m.keyframe_row(0).is_none() {
        return fail(
            ErrorKind::InvalidInput,
            "m: expected the map's first keyframe (id 0)",
        );
    }
    let rows: Vec<usize> = (0..m.keypoints()).filter(|&r| m.kp_point[r] >= 0).collect();
    let cameras: Vec<usize> = rows.iter().map(|&r| m.keyframe_of(r)).collect();
    let mut point_ids = Vec::with_capacity(rows.len());
    for &r in &rows {
        let Some(p) = m.point_row(m.kp_point[r]) else {
            return fail(
                ErrorKind::InvalidInput,
                "map: a keypoint observes a point not in it",
            );
        };
        point_ids.push(p);
    }
    let pixels: Vec<Vector2<f64>> = rows
        .iter()
        .map(|&r| Vector2::new(m.kp_xy[r][0], m.kp_xy[r][1]))
        .collect();
    let right: Vec<f64> = rows.iter().map(|&r| m.kp_right[r]).collect();
    let information: Vec<f64> = rows
        .iter()
        .map(|&r| 1.0 / sigma2(m, m.kp_level[r]))
        .collect();
    let poses: Vec<Matrix4<f64>> = m.kf_pose.iter().map(sim3::inverse).collect();
    let fixed: Vec<usize> = (0..m.keyframes()).filter(|&k| m.kf_id[k] == 0).collect();
    let adjusted = adjust(
        &poses,
        &m.pt_xyz,
        &graphs::Observations {
            cameras: &cameras,
            point_ids: &point_ids,
            pixels: &pixels,
            right: &right,
            information: &information,
            huber: &[],
        },
        camera,
        &fixed,
        options.loops.global_iterations,
        stop,
    )?;
    m.kf_pose = adjusted.poses.iter().map(sim3::inverse).collect();
    m.pt_xyz = adjusted.points;
    let all: Vec<usize> = (0..m.points()).collect();
    map::refresh_points(m, &all)?;
    Ok(adjusted.summary)
}

/// How a global BA's result was carried onto the map as it has become
/// (`slam_it.edge.loops.Propagated`, the map aside: it changes in place).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Propagated {
    /// Its keyframes the global BA solved, set to their result.
    pub adjusted: usize,
    /// Its keyframes made since, moved with their parents.
    pub propagated: usize,
    /// Its points the global BA solved, set to their result.
    pub points: usize,
    /// Its points made since, moved with their reference keyframes.
    pub carried: usize,
}

/// How a global BA in a thread of its own went (`slam_it.edge.loops.GlobalSummary`), as a run
/// records it on the frame it ended on.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobalSummary {
    /// Its generation: the loops corrected before it in the run.
    pub generation: u64,
    /// Whether its result was carried onto the map; not if a newer loop superseded it, or the
    /// map was lost, first.
    pub applied: bool,
    /// Its solve.
    pub solve: graphs::Summary,
    /// How its result was carried, when it was.
    pub propagation: Option<Propagated>,
}

/// Carries a global BA's result, solved on an earlier copy of the map, onto the map as it is
/// now (`slam_it.edge.loops.propagate_global_ba`, ORB-SLAM's `RunGlobalBundleAdjustment` after
/// its solve): walking the spanning tree from its roots, each keyframe the BA solved takes its
/// solved pose, and each made since moves with its parent
/// (`T_child' = T_child T_parent^-1 T_parent'`); each point the BA solved takes its solved
/// position, and each made since moves with its reference keyframe (`X' = T_ref'^-1 T_ref X`).
/// A keyframe the walk does not reach, or a point whose reference keyframe is gone, stays where
/// it is. Every point is refreshed after.
///
/// # Arguments
///
/// * `now` - The map as it is now, changed in place.
/// * `solved` - The copy the global BA solved, moved to its result.
///
/// # Returns
///
/// How many keyframes and points went each way.
///
/// # Errors
///
/// A refresh's.
pub fn propagate_global_ba(now: &mut Map, solved: &Map) -> Result<Propagated> {
    let mut poses = now.kf_pose.clone();
    let mut rows_of = BTreeMap::new();
    let mut children: BTreeMap<i64, Vec<usize>> = BTreeMap::new();
    for k in 0..now.keyframes() {
        rows_of.insert(now.kf_id[k], k);
        children.entry(now.kf_parent[k]).or_default().push(k);
    }
    let mut out = Propagated::default();
    let mut walk: VecDeque<usize> = children.get(&-1).cloned().unwrap_or_default().into();
    while let Some(k) = walk.pop_front() {
        if let Some(from) = solved.keyframe_row(now.kf_id[k]) {
            poses[k] = solved.kf_pose[from];
            out.adjusted += 1;
        } else if let Some(&p) = rows_of.get(&now.kf_parent[k]) {
            poses[k] = now.kf_pose[k] * sim3::inverse(&now.kf_pose[p]) * poses[p];
            out.propagated += 1;
        }
        if let Some(below) = children.get(&now.kf_id[k]) {
            walk.extend(below);
        }
    }
    for p in 0..now.points() {
        if let Some(from) = solved.point_row(now.pt_id[p]) {
            now.pt_xyz[p] = solved.pt_xyz[from];
            out.points += 1;
            continue;
        }
        let Some(&reference) = rows_of.get(&now.pt_reference_kf[p]) else {
            continue;
        };
        let local = to_camera(&now.kf_pose[reference], &now.pt_xyz[p]);
        now.pt_xyz[p] = to_camera(&sim3::inverse(&poses[reference]), &local);
        out.carried += 1;
    }
    now.kf_pose = poses;
    let all: Vec<usize> = (0..now.points()).collect();
    map::refresh_points(now, &all)?;
    Ok(out)
}

/// Sets each keyframe to its optimised pose, and moves each point with its keyframe
/// (`slam_it.edge.loops._move_by_graph`): a point moved by the loop goes with the keyframe that
/// moved it, any other with its reference keyframe.
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `start` - Each keyframe's `S_iw` the graph started from, by row.
/// * `optimised` - Each one's optimised `S_iw`, by row.
/// * `moved_by` - Each moved point's id to the row of the keyframe that moved it.
///
/// # Errors
///
/// The refresh's, which an observed point never meets.
fn move_by_graph(
    m: &mut Map,
    start: &[Matrix4<f64>],
    optimised: &[Matrix4<f64>],
    moved_by: &BTreeMap<i64, usize>,
) -> Result<()> {
    for p in 0..m.points() {
        let r = match moved_by.get(&m.pt_id[p]) {
            Some(&r) => r,
            None => match m.keyframe_row(m.pt_reference_kf[p]) {
                Some(r) => r,
                None => continue,
            },
        };
        let moving = sim3::inverse(&optimised[r]) * start[r];
        m.pt_xyz[p] = carried(&moving, &m.pt_xyz[p]);
    }
    m.kf_pose = optimised.iter().map(rigid_of).collect();
    let all: Vec<usize> = (0..m.points()).collect();
    map::refresh_points(m, &all)
}

/// Corrects a loop closed at a keyframe (`slam_it.edge.loops.correct`, ORB-SLAM3's
/// `CorrectLoop`).
///
/// # Arguments
///
/// * `m` - The map, holding both keyframes, changed in place.
/// * `camera` - The camera.
/// * `closed` - The loop.
/// * `mode` - The pipeline's mode: the essential graph is Sim(3) but in stereo and RGB-D.
/// * `options` - The settings.
///
/// # Returns
///
/// How it went.
///
/// # Errors
///
/// `InvalidInput` if either keyframe is not in the map, or the matched points are not one per
/// keypoint of the current keyframe; a stage's.
pub fn correct(
    m: &mut Map,
    camera: &Camera,
    closed: &Loop,
    mode: Mode,
    options: &Options,
) -> Result<CorrectionSummary> {
    let k = keyframe_row(m, closed.current)?;
    let matched = keyframe_row(m, closed.matched)?;
    check_keypoint_ids(m, k, &closed.matched_points, "loop.matched_points")?;
    let fixed_scale = mode != Mode::Monocular;
    let linked_rows = links(m, k, options)?;
    let mut moved_rows = vec![k];
    moved_rows.extend(&linked_rows);
    let before = m.kf_pose.clone();
    let mut corrected = BTreeMap::new();
    corrected.insert(k, closed.similarity);
    let to_world = sim3::inverse(&m.kf_pose[k]);
    for &r in &linked_rows {
        corrected.insert(r, m.kf_pose[r] * to_world * closed.similarity);
    }
    let mut previous = BTreeMap::new();
    for &r in &moved_rows {
        previous.insert(r, linked(m, r, options)?);
    }
    let moved_by = move_keyframes(m, &moved_rows, &corrected)?;
    let replaced = replace_matched(m, closed.current, &closed.matched_points)?;
    // Moving, replacing and fusing change points and the moved keyframes' poses, but never add,
    // remove or reorder a keyframe: every keyframe keeps its row from here to the graph, so the
    // rows found above still name them.
    let mut fused = 0;
    for &r in &moved_rows {
        fused += fuse_by_similarity(m, camera, r, &corrected[&r], &closed.points, options)?;
    }
    let moving: BTreeSet<usize> = moved_rows.iter().copied().collect();
    let mut loop_links = BTreeMap::new();
    for &r in &moved_rows {
        let new: BTreeSet<usize> = linked(m, r, options)?
            .difference(&previous[&r])
            .copied()
            .filter(|n| !moving.contains(n))
            .collect();
        loop_links.insert(r, new);
    }
    let mut start = m.kf_pose.clone();
    for &r in &moved_rows {
        start[r] = corrected[&r];
    }
    let essential = essential_graph(
        m,
        &start,
        &before,
        &loop_links,
        (k, matched),
        &[matched],
        fixed_scale,
        options,
    )?;
    let join = Join {
        add: map::add_loop_edge,
        ends: (closed.matched, closed.current),
    };
    let global_ba = finish_correction(m, &start, &essential, &moved_by, &join, camera, options)?;
    Ok(CorrectionSummary {
        current: closed.current,
        matched: closed.matched,
        similarity: closed.similarity,
        corrected: moved_rows.len(),
        moved: moved_by.len(),
        replaced,
        fused,
        links: loop_links.values().map(BTreeSet::len).sum(),
        edges: essential.edges,
        essential: essential.summary,
        global_ba,
    })
}

/// How a correction joins its two keyframes: the map's operation that adds the edge, and its
/// ends' ids.
struct Join {
    /// `map::add_loop_edge` or `map::add_merge_edge`.
    add: fn(&mut Map, i64, i64) -> Result<()>,
    /// The two keyframes' ids.
    ends: (i64, i64),
}

/// A correction's last steps (`CorrectLoop`'s and `MergeLocal`'s): each keyframe to its
/// optimised pose and each point with its keyframe, the two keyframes joined, then global BA if
/// `loops.global_ba` asks.
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `start` - Each keyframe's `S_iw` the graph started from, by row.
/// * `essential` - The essential graph's solution.
/// * `moved_by` - Each point the correction moved, and the keyframe that moved it.
/// * `join` - The edge joining the two keyframes.
/// * `camera` - The camera.
/// * `options` - The settings.
///
/// # Returns
///
/// Global BA's solve, or `None` with `loops.global_ba` off.
///
/// # Errors
///
/// The map's, or global BA's.
fn finish_correction(
    m: &mut Map,
    start: &[Matrix4<f64>],
    essential: &Essential,
    moved_by: &BTreeMap<i64, usize>,
    join: &Join,
    camera: &Camera,
    options: &Options,
) -> Result<Option<graphs::Summary>> {
    move_by_graph(m, start, &essential.poses, moved_by)?;
    (join.add)(m, join.ends.0, join.ends.1)?;
    if options.loops.global_ba {
        Ok(Some(global_bundle_adjust(
            m,
            camera,
            options,
            &Stop::never(),
        )?))
    } else {
        Ok(None)
    }
}

// --- merging -------------------------------------------------------------------------------

/// How a merge's welding BA went (`slam_it.edge.loops.WeldSummary`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WeldSummary {
    /// The keyframes optimised.
    pub keyframes: usize,
    /// The keyframes held fixed.
    pub held: usize,
    /// The points optimised.
    pub points: usize,
    /// The observations weighed.
    pub observations: usize,
    /// Those failing the chi-square test, or behind their camera, after the first solve.
    pub outliers: usize,
    /// Those failing it after the second, forgotten.
    pub forgotten: usize,
    /// The first solve, with a Huber loss.
    pub first: graphs::Summary,
    /// The second, with none, if any observation was left for it.
    pub second: Option<graphs::Summary>,
}

/// How a merge went (`slam_it.edge.loops.MergeSummary`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MergeSummary {
    /// The number of the map the active one was merged into.
    pub map: i64,
    /// The id, in the merged map, of the keyframe the merge was found on.
    pub current: i64,
    /// The id of the keyframe it matched.
    pub matched: i64,
    /// The current keyframe's `S_cw` in the matched map's world, as the merge placed it.
    pub similarity: Matrix4<f64>,
    /// The active side's welding window's keyframes.
    pub window: usize,
    /// The matched side's.
    pub matched_window: usize,
    /// The matched window's points fused into the active window's keyframes.
    pub fused: usize,
    /// The welding BA.
    pub weld: WeldSummary,
    /// The essential graph's edges.
    pub edges: usize,
    /// Its solve.
    pub essential: graphs::Summary,
    /// Global BA's solve, if it ran.
    pub global_ba: Option<graphs::Summary>,
}

/// Two maps merged (`slam_it.edge.loops.Merged`).
#[derive(Debug, Clone)]
pub struct Merged {
    /// The merged map: the matched map's, then the active map's, each id of the active map's
    /// moved on by an offset.
    pub map: Map,
    /// What each of the active map's keyframe ids, live or archived, became: the id plus this.
    pub keyframe_offset: i64,
    /// The same for its point ids.
    pub point_offset: i64,
    /// How it went.
    pub summary: MergeSummary,
}

/// `G`, the similarity carrying a merge's active map into the matched map's world
/// (`slam_it.edge.loops.merge_transform`): `S⁻¹ T`, `S` the merge's similarity placing the
/// current keyframe in the matched map's world, `T` its pose in the active map's.
///
/// # Arguments
///
/// * `active` - The active map, holding the merge's current keyframe.
/// * `found` - The merge.
///
/// # Returns
///
/// `G`.
///
/// # Errors
///
/// `InvalidInput` if the active map does not hold the current keyframe.
pub fn merge_transform(active: &Map, found: &Merge) -> Result<Matrix4<f64>> {
    let row = keyframe_row(active, found.found.current)?;
    Ok(sim3::inverse(&found.found.similarity) * active.kf_pose[row])
}

/// A pose scaled into a similarity: `[[s R, s t], [0, 1]]`.
///
/// # Arguments
///
/// * `pose` - The pose.
/// * `scale` - `s`.
///
/// # Returns
///
/// The similarity.
fn scaled(pose: &Matrix4<f64>, scale: f64) -> Matrix4<f64> {
    let mut out = *pose;
    for c in 0..4 {
        for r in 0..3 {
            out[(r, c)] *= scale;
        }
    }
    out
}

/// Carries a map into another world by a similarity `G` on its own
/// (`slam_it.edge.loops._moved_into`): each keyframe's pose `rigid_of(T G⁻¹)`, each point
/// `G p`, its archive with it, every observed point refreshed.
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `to_world` - `G`.
///
/// # Errors
///
/// The refresh's.
fn moved_into(m: &mut Map, to_world: &Matrix4<f64>) -> Result<()> {
    let back = sim3::inverse(to_world);
    let scale = scale_of(&back);
    for pose in m.kf_pose.iter_mut().chain(m.gone_kf_pose.iter_mut()) {
        *pose = rigid_of(&(*pose * back));
    }
    for x in m.pt_xyz.iter_mut().chain(m.gone_pt_xyz.iter_mut()) {
        *x = carried(to_world, x);
    }
    for relative in &mut m.gone_kf_relative {
        for r in 0..3 {
            relative[(r, 3)] /= scale;
        }
    }
    let seen: Vec<usize> = (0..m.points())
        .filter(|&p| !m.pt_observations[p].is_empty())
        .collect();
    map::refresh_points(m, &seen)
}

/// Ids moved on by an offset, -1 (none) kept.
///
/// # Arguments
///
/// * `ids` - The ids.
/// * `offset` - The offset.
///
/// # Returns
///
/// The ids moved on.
fn offset_ids(ids: &[i64], offset: i64) -> Vec<i64> {
    ids.iter()
        .map(|&id| if id >= 0 { id + offset } else { id })
        .collect()
}

/// Takes `other`'s keyframes, keypoints, points and archive into `into`, after its own
/// (`slam_it.edge.loops._absorb`): every id of `other`'s moved on by its offset.
///
/// # Arguments
///
/// * `into` - The map taking them, changed in place.
/// * `other` - The map taken, already in `into`'s world.
/// * `keyframe_offset` - The keyframe ids' offset.
/// * `point_offset` - The point ids' offset.
fn absorb(into: &mut Map, other: &Map, keyframe_offset: i64, point_offset: i64) {
    let (kf, pt) = (keyframe_offset, point_offset);
    let keypoints = into.keypoints();
    into.kf_id.extend(offset_ids(&other.kf_id, kf));
    into.kf_time.extend(&other.kf_time);
    into.kf_pose.extend(&other.kf_pose);
    into.kf_parent.extend(offset_ids(&other.kf_parent, kf));
    into.kf_start
        .extend(other.kf_start[1..].iter().map(|&s| s + keypoints));
    into.kp_xy.extend(&other.kp_xy);
    into.kp_level.extend(&other.kp_level);
    into.kp_angle.extend(&other.kp_angle);
    into.kp_descriptor.extend(&other.kp_descriptor);
    into.kp_bearing.extend(&other.kp_bearing);
    into.kp_depth.extend(&other.kp_depth);
    into.kp_right.extend(&other.kp_right);
    into.kp_point.extend(offset_ids(&other.kp_point, pt));
    into.pt_id.extend(offset_ids(&other.pt_id, pt));
    into.pt_xyz.extend(&other.pt_xyz);
    into.pt_descriptor.extend(&other.pt_descriptor);
    into.pt_normal.extend(&other.pt_normal);
    into.pt_min_distance.extend(&other.pt_min_distance);
    into.pt_max_distance.extend(&other.pt_max_distance);
    into.pt_visible.extend(&other.pt_visible);
    into.pt_found.extend(&other.pt_found);
    into.pt_first_kf.extend(offset_ids(&other.pt_first_kf, kf));
    into.pt_reference_kf
        .extend(offset_ids(&other.pt_reference_kf, kf));
    into.next_kf_id = kf + other.next_kf_id;
    into.next_pt_id = pt + other.next_pt_id;
    into.gone_pt_id.extend(offset_ids(&other.gone_pt_id, pt));
    into.gone_pt_xyz.extend(&other.gone_pt_xyz);
    into.gone_pt_how.extend(&other.gone_pt_how);
    into.gone_kf_id.extend(offset_ids(&other.gone_kf_id, kf));
    into.gone_kf_time.extend(&other.gone_kf_time);
    into.gone_kf_pose.extend(&other.gone_kf_pose);
    into.gone_kf_parent
        .extend(offset_ids(&other.gone_kf_parent, kf));
    into.gone_kf_relative.extend(&other.gone_kf_relative);
    into.loop_edges
        .extend(other.loop_edges.iter().map(|&[a, b]| [a + kf, b + kf]));
    into.merge_edges
        .extend(other.merge_edges.iter().map(|&[a, b]| [a + kf, b + kf]));
    map::index_observations(into);
}

/// Re-roots the active side's spanning tree at its current keyframe, hung under the matched
/// one (`slam_it.edge.loops._rehang`).
///
/// # Arguments
///
/// * `m` - The merged map, changed in place.
/// * `current` - The current keyframe's id.
/// * `matched` - The matched keyframe's id.
///
/// # Errors
///
/// `InvalidInput` for a keyframe of the chain the map does not hold.
fn rehang(m: &mut Map, current: i64, matched: i64) -> Result<()> {
    let mut chain = vec![current];
    loop {
        let parent = m.kf_parent[keyframe_row(m, chain[chain.len() - 1])?];
        if parent < 0 {
            break;
        }
        chain.push(parent);
    }
    for pair in chain.windows(2) {
        let row = keyframe_row(m, pair[1])?;
        m.kf_parent[row] = pair[0];
    }
    let row = keyframe_row(m, current)?;
    m.kf_parent[row] = matched;
    Ok(())
}

/// A welding window's keyframe rows (`slam_it.edge.loops._weld_window`): the keyframe and its
/// best `loops.weld_neighbours` links; while it holds fewer, each member's best
/// `loops.weld_widen` links not yet in it, at most `loops.weld_rounds` times.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `options` - The settings.
///
/// # Returns
///
/// The rows, in the order taken in.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn weld_window(m: &Map, k: usize, options: &Options) -> Result<Vec<usize>> {
    let settings = &options.loops;
    let mut window = vec![k];
    let mut first = links(m, k, options)?;
    first.truncate(settings.weld_neighbours);
    window.extend(first);
    for _ in 0..settings.weld_rounds {
        if window.len() >= settings.weld_neighbours {
            break;
        }
        let mut inside: BTreeSet<usize> = window.iter().copied().collect();
        let mut added = Vec::new();
        for &r in &window {
            let mut theirs = links(m, r, options)?;
            theirs.truncate(settings.weld_widen);
            for other in theirs {
                if inside.insert(other) {
                    added.push(other);
                }
            }
        }
        if added.is_empty() {
            break;
        }
        window.extend(added);
    }
    Ok(window)
}

/// A welding BA's problem (`slam_it.edge.loops._Bundle`): its cameras and points, and each
/// observation of them.
struct Weld {
    /// The cameras' keyframe rows: the windows', ascending, then those held outside them.
    cameras: Vec<usize>,
    /// The held cameras, as indices into `cameras`.
    fixed: Vec<usize>,
    /// The points' rows, ascending.
    points: Vec<usize>,
    /// Each observation's keypoint row, ascending.
    rows: Vec<usize>,
    /// Each observation's camera, an index into `cameras`.
    camera: Vec<usize>,
    /// Each observation's point, an index into `points`.
    point: Vec<usize>,
    /// Each observation's pixel.
    xy: Vec<[f64; 2]>,
    /// Its right column, or NaN.
    right: Vec<f64>,
    /// Its level's variance.
    variance: Vec<f64>,
    /// Its chi-square threshold.
    limits: Vec<f64>,
}

/// Gathers a welding BA's problem (`slam_it.edge.loops._weld_problem`): both windows'
/// keyframes free with every point they observe; the matched map's keyframes outside them that
/// observe those points held, or else the matched window.
///
/// # Arguments
///
/// * `m` - The merged map.
/// * `windows` - The active side's window's rows, and the matched side's.
/// * `keyframe_offset` - The first keyframe id that came from the active map.
/// * `options` - The settings.
///
/// # Returns
///
/// The problem.
fn weld_problem(
    m: &Map,
    windows: (&[usize], &[usize]),
    keyframe_offset: i64,
    options: &Options,
) -> Weld {
    let (window, matched_window) = windows;
    let free: BTreeSet<usize> = window.iter().chain(matched_window).copied().collect();
    let mut cameras: Vec<usize> = free.iter().copied().collect();
    let points = m.observed_by(&cameras);
    let mut observing: Vec<usize> = points
        .iter()
        .flat_map(|&p| m.pt_observations[p].iter().copied())
        .collect();
    observing.sort_unstable();
    let outside: BTreeSet<usize> = observing
        .iter()
        .map(|&row| m.keyframe_of(row))
        .filter(|k| !free.contains(k))
        .collect();
    let held: Vec<usize> = outside
        .into_iter()
        .filter(|&k| m.kf_id[k] < keyframe_offset)
        .collect();
    cameras.extend(&held);
    let mut fixed_rows = held;
    if fixed_rows.is_empty() {
        fixed_rows = matched_window.to_vec();
        fixed_rows.sort_unstable();
    }
    let index: BTreeMap<usize, usize> = cameras.iter().enumerate().map(|(i, &k)| (k, i)).collect();
    let mut out = Weld {
        fixed: fixed_rows.iter().map(|k| index[k]).collect(),
        cameras: Vec::new(),
        points: Vec::new(),
        rows: Vec::new(),
        camera: Vec::new(),
        point: Vec::new(),
        xy: Vec::new(),
        right: Vec::new(),
        variance: Vec::new(),
        limits: Vec::new(),
    };
    for row in observing {
        let Some(&at) = index.get(&m.keyframe_of(row)) else {
            continue;
        };
        let Some(p) = m.point_row(m.kp_point[row]) else {
            continue;
        };
        out.rows.push(row);
        out.camera.push(at);
        out.point.push(points.partition_point(|&q| q < p));
        out.xy.push(m.kp_xy[row]);
        out.right.push(m.kp_right[row]);
        out.variance.push(sigma2(m, m.kp_level[row]));
        out.limits.push(threshold(m.kp_right[row], options));
    }
    out.cameras = cameras;
    out.points = points;
    out
}

/// The welding BA's state: each camera's `T_world_cam`, each point.
struct WeldState {
    /// The cameras' poses.
    poses: Vec<Matrix4<f64>>,
    /// The points.
    xyz: Vec<Vector3<f64>>,
}

/// Solves the welding BA on the marked observations (`slam_it.edge.loops._weld_solve`): only the
/// cameras and points they reach take part, with the held cameras.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `camera` - The camera.
/// * `state` - The state, updated in place.
/// * `used` - Which observations take part.
/// * `robust` - With a Huber loss at each one's threshold's root.
/// * `iterations` - The solve's iterations.
///
/// # Returns
///
/// The solve's summary.
///
/// # Errors
///
/// GTSAM's.
fn weld_solve(
    problem: &Weld,
    camera: &Camera,
    state: &mut WeldState,
    used: &[bool],
    robust: bool,
    iterations: usize,
) -> Result<graphs::Summary> {
    let mut cameras: BTreeSet<usize> = problem.fixed.iter().copied().collect();
    let mut points = BTreeSet::new();
    for (i, &taking) in used.iter().enumerate() {
        if taking {
            cameras.insert(problem.camera[i]);
            points.insert(problem.point[i]);
        }
    }
    let used_cameras: Vec<usize> = cameras.into_iter().collect();
    let used_points: Vec<usize> = points.into_iter().collect();
    let place = |values: &[usize], value: usize| values.partition_point(|&v| v < value);
    let (mut which, mut whose, mut pixels) = (Vec::new(), Vec::new(), Vec::new());
    let (mut right, mut information, mut huber) = (Vec::new(), Vec::new(), Vec::new());
    for (i, &taking) in used.iter().enumerate() {
        if !taking {
            continue;
        }
        which.push(place(&used_cameras, problem.camera[i]));
        whose.push(place(&used_points, problem.point[i]));
        pixels.push(Vector2::new(problem.xy[i][0], problem.xy[i][1]));
        right.push(problem.right[i]);
        information.push(1.0 / problem.variance[i]);
        if robust {
            huber.push(problem.limits[i].sqrt());
        }
    }
    let poses: Vec<Matrix4<f64>> = used_cameras.iter().map(|&c| state.poses[c]).collect();
    let start: Vec<Vector3<f64>> = used_points.iter().map(|&p| state.xyz[p]).collect();
    let fixed: Vec<usize> = problem
        .fixed
        .iter()
        .map(|&c| place(&used_cameras, c))
        .collect();
    let adjusted = adjust(
        &poses,
        &start,
        &graphs::Observations {
            cameras: &which,
            point_ids: &whose,
            pixels: &pixels,
            right: &right,
            information: &information,
            huber: &huber,
        },
        camera,
        &fixed,
        iterations,
        &Stop::never(),
    )?;
    for (c, pose) in used_cameras.iter().zip(&adjusted.poses) {
        state.poses[*c] = *pose;
    }
    for (p, xyz) in used_points.iter().zip(&adjusted.points) {
        state.xyz[*p] = *xyz;
    }
    Ok(adjusted.summary)
}

/// Each observation's chi-square error at a state (`slam_it.edge.loops._weld_errors`).
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `camera` - The camera.
/// * `state` - The state.
///
/// # Returns
///
/// The errors.
///
/// # Errors
///
/// The residuals'.
fn weld_errors(problem: &Weld, camera: &Camera, state: &WeldState) -> Result<Vec<f64>> {
    let local: Vec<Vector3<f64>> = problem
        .camera
        .iter()
        .zip(&problem.point)
        .map(|(&c, &p)| to_camera(&sim3::inverse(&state.poses[c]), &state.xyz[p]))
        .collect();
    chi2(
        camera,
        &local,
        &problem.xy,
        &problem.right,
        &problem.variance,
    )
}

/// Bundle-adjusts the welding windows (`slam_it.edge.loops._weld`): the problem solved as
/// ORB-SLAM's local BA solves its own, a Huber loss first, the outliers left out of a second
/// solve with none, those still failing forgotten, the points refreshed.
///
/// # Arguments
///
/// * `m` - The merged map, changed in place.
/// * `camera` - The camera.
/// * `windows` - The active side's window's rows, and the matched side's.
/// * `keyframe_offset` - The first keyframe id that came from the active map.
/// * `options` - The settings.
///
/// # Returns
///
/// How it went.
///
/// # Errors
///
/// GTSAM's or the map's.
fn weld(
    m: &mut Map,
    camera: &Camera,
    windows: (&[usize], &[usize]),
    keyframe_offset: i64,
    options: &Options,
) -> Result<WeldSummary> {
    let problem = weld_problem(m, windows, keyframe_offset, options);
    let mut state = WeldState {
        poses: problem
            .cameras
            .iter()
            .map(|&r| sim3::inverse(&m.kf_pose[r]))
            .collect(),
        xyz: problem.points.iter().map(|&p| m.pt_xyz[p]).collect(),
    };
    let everything = vec![true; problem.rows.len()];
    let first = weld_solve(
        &problem,
        camera,
        &mut state,
        &everything,
        true,
        options.adjustment.iterations,
    )?;
    let inliers: Vec<bool> = weld_errors(&problem, camera, &state)?
        .iter()
        .zip(&problem.limits)
        .map(|(e, limit)| e <= limit)
        .collect();
    let second = if inliers.iter().any(|&inlier| inlier) {
        Some(weld_solve(
            &problem,
            camera,
            &mut state,
            &inliers,
            false,
            options.adjustment.refine_iterations,
        )?)
    } else {
        None
    };
    let failing: Vec<usize> = weld_errors(&problem, camera, &state)?
        .iter()
        .zip(&problem.limits)
        .zip(&problem.rows)
        .filter(|((e, limit), _)| !(*e <= *limit))
        .map(|(_, &row)| row)
        .collect();
    let held: BTreeSet<usize> = problem.fixed.iter().copied().collect();
    for (i, &r) in problem.cameras.iter().enumerate() {
        if !held.contains(&i) {
            m.kf_pose[r] = sim3::inverse(&state.poses[i]);
        }
    }
    for (&p, xyz) in problem.points.iter().zip(&state.xyz) {
        m.pt_xyz[p] = *xyz;
    }
    map::refresh_points(m, &problem.points)?;
    map::forget_observations(m, &failing)?;
    Ok(WeldSummary {
        keyframes: problem.cameras.len() - held.len(),
        held: held.len(),
        points: problem.points.len(),
        observations: problem.rows.len(),
        outliers: inliers.iter().filter(|&&inlier| !inlier).count(),
        forgotten: failing.len(),
        first,
        second,
    })
}

/// Refuses two maps whose pyramids differ: a merge's fusion matches one's points to the other's
/// keypoints by level.
///
/// # Arguments
///
/// * `active` - The active map.
/// * `matched` - The map it matched.
///
/// # Errors
///
/// `InvalidInput` for other levels or another scale factor.
#[allow(
    clippy::float_cmp,
    reason = "the two maps' pyramids are the same settings, equal exactly, as Python checks them"
)]
fn check_pyramids(active: &Map, matched: &Map) -> Result<()> {
    if active.levels != matched.levels || active.scale_factor != matched.scale_factor {
        return fail(
            ErrorKind::InvalidInput,
            "merge: the two maps' pyramids differ",
        );
    }
    Ok(())
}

/// Merges the active map into the one it matched (`slam_it.edge.loops.merge`, `MergeLocal` as
/// ORB-SLAM3's paper gives it; the Python module's docstring states the steps).
///
/// # Arguments
///
/// * `active` - The active map, holding the merge's current keyframe.
/// * `matched` - The map it matched, holding the merge's matched keyframe.
/// * `camera` - The camera.
/// * `found` - The merge.
/// * `mode` - The pipeline's mode: the similarity's scale is used but in stereo and RGB-D.
/// * `options` - The settings.
///
/// # Returns
///
/// The merged map, the offsets that carried the active map's ids, and how it went.
///
/// # Errors
///
/// `InvalidInput` for a keyframe its map does not hold, or maps of different pyramids;
/// `NotConverged` for GTSAM's failure solving the welding BA, the essential graph or global BA;
/// a stage's.
pub fn merge(
    active: &Map,
    matched: &Map,
    camera: &Camera,
    found: &Merge,
    mode: Mode,
    options: &Options,
) -> Result<Merged> {
    let region = &found.found;
    let to_matched = merge_transform(active, found)?;
    keyframe_row(matched, region.matched)?;
    check_pyramids(active, matched)?;
    let fixed_scale = mode != Mode::Monocular;
    let scale = if fixed_scale {
        1.0
    } else {
        scale_of(&region.similarity)
    };
    let (keyframe_offset, point_offset) = (matched.next_kf_id, matched.next_pt_id);
    let mut moved = active.clone();
    moved_into(&mut moved, &to_matched)?;
    let mut m = matched.clone();
    absorb(&mut m, &moved, keyframe_offset, point_offset);
    let current = region.current + keyframe_offset;
    rehang(&mut m, current, region.matched)?;
    let k = keyframe_row(&m, current)?;
    let km = keyframe_row(&m, region.matched)?;
    let window = weld_window(&m, k, options)?;
    let matched_window = weld_window(&m, km, options)?;
    let points: Vec<i64> = keyframe_points(&m, &matched_window)
        .iter()
        .map(|&p| m.pt_id[p])
        .collect();
    let mut fused = 0;
    for &r in &window {
        // Fusion erases points, never keyframes: the rows hold.
        let pose = m.kf_pose[r];
        fused += fuse_by_similarity(&mut m, camera, r, &pose, &points, options)?;
    }
    let before = m.kf_pose.clone();
    let welded = weld(
        &mut m,
        camera,
        (&window, &matched_window),
        keyframe_offset,
        options,
    )?;
    // The active side's nodes carry its own units, as its edges measured before were.
    let absorbed: Vec<bool> = m.kf_id.iter().map(|&id| id >= keyframe_offset).collect();
    let in_units = |poses: &[Matrix4<f64>]| -> Vec<Matrix4<f64>> {
        poses
            .iter()
            .zip(&absorbed)
            .map(|(pose, &own)| if own { scaled(pose, scale) } else { *pose })
            .collect()
    };
    let start = in_units(&m.kf_pose);
    let prior = in_units(&before);
    let held: BTreeSet<usize> = window.iter().chain(&matched_window).copied().collect();
    let held: Vec<usize> = held.into_iter().collect();
    let essential = essential_graph(
        &m,
        &start,
        &prior,
        &BTreeMap::new(),
        (k, km),
        &held,
        fixed_scale,
        options,
    )?;
    let join = Join {
        add: map::add_merge_edge,
        ends: (region.matched, current),
    };
    let global_ba = finish_correction(
        &mut m,
        &start,
        &essential,
        &BTreeMap::new(),
        &join,
        camera,
        options,
    )?;
    Ok(Merged {
        map: m,
        keyframe_offset,
        point_offset,
        summary: MergeSummary {
            map: found.map,
            current,
            matched: region.matched,
            similarity: region.similarity,
            window: window.len(),
            matched_window: matched_window.len(),
            fused,
            weld: welded,
            edges: essential.edges,
            essential: essential.summary,
            global_ba,
        },
    })
}
