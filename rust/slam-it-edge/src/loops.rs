//! Loop closing: the Rust core's mirror of `slam_it.edge.loops` (ORB-SLAM3's `LoopClosing`),
//! whose docstring states the method: detection (`NewDetectCommonRegions`), a loop being
//! confirmed carried from keyframe to keyframe, and the correction (`CorrectLoop`) with its
//! essential graph and global bundle adjustment, and where it departs from ORB-SLAM3's source.
//!
//! Similarities are ORB-SLAM's `S_cw`, `[[s R, t], [0, 1]]`; a keyframe's own pose is one with
//! `s = 1`. A search projects through the rigid `(R, t / s)`.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{Matrix4, Vector2, Vector3};
use slam_it_foundations::camera::MIN_DEPTH;
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::graphs;
use slam_it_foundations::matching;
use slam_it_foundations::orb;
use slam_it_foundations::ransac;
use slam_it_foundations::sim3;

use crate::covisibility;
use crate::database::{self, Database};
use crate::frames::Camera;
use crate::map::{self, Map, camera_centre, keyframe_row, predict_level, to_camera, viewing_cos};
use crate::options::{Mode, Options};
use crate::residuals::sigma2;

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
/// * `m` - The map.
/// * `p` - The point's row.
/// * `near` - The keypoints' rows, at least one.
///
/// # Returns
///
/// Its index in `near`, and its distance.
fn nearest(m: &Map, p: usize, near: &[usize]) -> (usize, i64) {
    let mut best = (0, i64::MAX);
    for (i, &r) in near.iter().enumerate() {
        let d = orb::distance(&m.pt_descriptor[p], &m.kp_descriptor[r]);
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
#[allow(clippy::cast_precision_loss, reason = "levels are at most MAX_LEVELS")]
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
    let start = m.kf_start[k];
    let mut out = vec![-1; m.kf_start[k + 1] - start];
    let seen = in_view(m, camera, similarity, points, options)?;
    for ((&p, pixel), &level) in seen.rows.iter().zip(&seen.pixels).zip(&seen.levels) {
        let radius = search.window * m.scale_factor.powf(level as f64);
        let near: Vec<usize> = candidates_near(m, k, pixel, radius, level)
            .into_iter()
            .filter(|&r| out[r - start] < 0)
            .collect();
        if near.is_empty() {
            continue;
        }
        let (best, distance) = nearest(m, p, &near);
        if distance <= search.distance {
            out[near[best] - start] = m.pt_id[p];
        }
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
    let side = |k: usize| -> Result<(std::ops::Range<usize>, Vec<usize>, Vec<usize>)> {
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
    };
    let (rows1, own1, nodes1) = side(k1)?;
    let (_, own2, nodes2) = side(k2)?;
    let descriptors = |own: &[usize]| own.iter().map(|&r| m.kp_descriptor[r]).collect::<Vec<_>>();
    let angles = |own: &[usize]| own.iter().map(|&r| m.kp_angle[r]).collect::<Vec<_>>();
    let found = matching::search_by_bow(
        &descriptors(&own1),
        &nodes1,
        &descriptors(&own2),
        &nodes2,
        &angles(&own1),
        &angles(&own2),
        &matching::Options {
            max_distance: options.matching.th_low,
            ratio: options.loops.ratio,
            check_orientation: true,
        },
    )?;
    let mut out = vec![-1; rows1.len()];
    for pair in &found {
        out[own1[pair.first] - rows1.start] = m.kp_point[own2[pair.second]];
    }
    Ok(out)
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
        let (Some(own), Some(partner)) = (m.point_row(m.kp_point[row]), m.point_row(matched[i]))
        else {
            continue;
        };
        let point1 = to_camera(&m.kf_pose[k1], &m.pt_xyz[own]);
        let point2 = to_camera(&m.kf_pose[k2], &m.pt_xyz[partner]);
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
    let rows2 = m.kf_start[k2]..m.kf_start[k2 + 1];
    for (&row, &partner) in rows.iter().zip(&partners) {
        let id = m.pt_id[partner];
        let (pixel, level) = rows2
            .clone()
            .find(|&r| m.kp_point[r] == id)
            .map_or((Vector2::repeat(f64::NAN), 0), |r| {
                (Vector2::new(m.kp_xy[r][0], m.kp_xy[r][1]), m.kp_level[r])
            });
        weighed
            .pixels1
            .push(Vector2::new(m.kp_xy[row][0], m.kp_xy[row][1]));
        weighed.pixels2.push(pixel);
        weighed
            .information
            .push([1.0 / sigma2(m, m.kp_level[row]), 1.0 / sigma2(m, level)]);
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
}

impl Outcome {
    /// Every outcome, in code order.
    pub const ALL: [Self; 6] = [
        Self::Skipped,
        Self::None,
        Self::Started,
        Self::Confirmed,
        Self::Closed,
        Self::Dropped,
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
/// (`slam_it.edge.loops._confirm_by_projection`, ORB-SLAM's `FindMatchesByProjection`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `matched` - The matched keyframe's row.
/// * `similarity` - The keyframe's `S_cw`.
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
    options: &Options,
) -> Result<(Vec<i64>, Vec<i64>)> {
    let points = group_points(m, matched, &linked(m, k, options)?, options)?;
    let found = search_by_similarity(
        m,
        camera,
        k,
        similarity,
        &points,
        Search {
            window: options.loops.confirm_window,
            distance: options.loops.confirm_distance,
        },
        options,
    )?;
    Ok((points.iter().map(|&p| m.pt_id[p]).collect(), found))
}

/// Tries to confirm the loop being confirmed on a later keyframe
/// (`slam_it.edge.loops._carry`, `DetectAndReffineSim3FromLastKF`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `pending` - The loop.
/// * `fixed_scale` - Whether the similarity is rigid.
/// * `options` - The settings.
///
/// # Returns
///
/// The loop as it stands after the keyframe, `None` once dropped, and how it went.
///
/// # Errors
///
/// `InvalidInput` for a keyframe of the loop the map does not hold; a search's or the
/// refinement's.
fn carry(
    m: &Map,
    camera: &Camera,
    k: usize,
    pending: &Pending,
    fixed_scale: bool,
    options: &Options,
) -> Result<(Option<Pending>, ConfirmationSummary)> {
    let settings = &options.loops;
    let matched = keyframe_row(m, pending.matched)?;
    let last = keyframe_row(m, pending.last)?;
    let moved = relative(&m.kf_pose[k], &m.kf_pose[last]) * pending.similarity;
    let (_, found) = confirm_by_projection(m, camera, k, matched, &moved, options)?;
    let mut summary = ConfirmationSummary {
        matched: pending.matched,
        projected: matched_count(&found),
        ..ConfirmationSummary::default()
    };
    let mut confirmed = None;
    if summary.projected >= settings.confirm_matches {
        let start = moved * sim3::inverse(&m.kf_pose[matched]);
        let refined = refine(m, camera, k, matched, &start, &found, fixed_scale, options)?;
        summary.refined = refined.map(|r| r.inliers);
        if let Some(refined) = refined
            && refined.inliers > settings.temporal_refined
        {
            let placed = refined.similarity * m.kf_pose[matched];
            let (points, again) = confirm_by_projection(m, camera, k, matched, &placed, options)?;
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
struct Merged {
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
/// * `m` - The map.
/// * `db` - The database.
/// * `k` - The keyframe's row.
/// * `group` - The group's rows, at least one.
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
    options: &Options,
) -> Result<Merged> {
    let size = m.kf_start[k + 1] - m.kf_start[k];
    let mut out = Merged {
        points: vec![-1; size],
        source: vec![usize::MAX; size],
        best: group[0],
    };
    let mut used = BTreeSet::new();
    let mut most = None;
    for &g in group {
        let found = match_by_word(m, db, k, g, options)?;
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
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `merged` - Its merged matches by word.
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
    merged: &Merged,
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
        let (Some(own), Some(partner)) = (m.point_row(m.kp_point[row]), m.point_row(id)) else {
            continue;
        };
        let g = merged.source[i];
        let at = (m.kf_start[g]..m.kf_start[g + 1])
            .find(|&r| m.kp_point[r] == id)
            .map_or(0, |r| m.kp_level[r]);
        let in_current = to_camera(&m.kf_pose[k], &m.pt_xyz[own]);
        let in_best = to_camera(&m.kf_pose[merged.best], &m.pt_xyz[partner]);
        if !(in_current.z > MIN_DEPTH && in_best.z > MIN_DEPTH) {
            continue;
        }
        points1.push(in_current);
        points2.push(in_best);
        sigma2_1.push(sigma2(m, m.kp_level[row]));
        sigma2_2.push(sigma2(m, at));
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
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `best` - The matched keyframe's row.
/// * `similarity` - The keyframe's `S_cw`.
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
    options: &Options,
) -> Result<usize> {
    let mut count = 0;
    for j in best_links(m, k, options)? {
        if count == options.loops.confirmations {
            break;
        }
        let moved = relative(&m.kf_pose[j], &m.kf_pose[k]) * similarity;
        let (_, found) = confirm_by_projection(m, camera, j, best, &moved, options)?;
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
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `best` - The best keyframe's row.
/// * `model` - `S_kb`.
/// * `summary` - The candidate's summary, each count reached written in.
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
    reason = "the map, its camera, the two keyframes, the start, the record"
)]
fn place(
    m: &Map,
    camera: &Camera,
    k: usize,
    best: usize,
    model: &Matrix4<f64>,
    summary: &mut CandidateSummary,
    fixed_scale: bool,
    options: &Options,
) -> Result<Option<Placed>> {
    let settings = &options.loops;
    let mut group = best_links(m, best, options)?;
    group.push(best);
    let points = keyframe_points(m, &group);
    let coarse = search_by_similarity(
        m,
        camera,
        k,
        &(model * m.kf_pose[best]),
        &points,
        Search {
            window: settings.coarse_window,
            distance: settings.coarse_distance,
        },
        options,
    )?;
    summary.projected = Some(matched_count(&coarse));
    if matched_count(&coarse) < settings.coarse_matches {
        return Ok(None);
    }
    let refined = refine(m, camera, k, best, model, &coarse, fixed_scale, options)?;
    summary.refined = refined.map(|r| r.inliers);
    let Some(refined) = refined.filter(|r| r.inliers >= settings.min_refined) else {
        return Ok(None);
    };
    let placed = refined.similarity * m.kf_pose[best];
    let fine = search_by_similarity(
        m,
        camera,
        k,
        &placed,
        &points,
        Search {
            window: settings.fine_window,
            distance: settings.fine_distance,
        },
        options,
    )?;
    summary.reprojected = Some(matched_count(&fine));
    if matched_count(&fine) < settings.fine_matches {
        return Ok(None);
    }
    Ok(Some((placed, points, fine)))
}

/// What the search of a keyframe needs besides its candidate: the map, its database and
/// camera, the keyframe, its links and the settings.
struct Searching<'a> {
    /// The map.
    m: &'a Map,
    /// Its database.
    db: &'a Database,
    /// The camera.
    camera: &'a Camera,
    /// The keyframe's row.
    k: usize,
    /// Its links' rows.
    linked: BTreeSet<usize>,
    /// Whether the similarity is rigid.
    fixed_scale: bool,
    /// The settings.
    options: &'a Options,
}

impl Searching<'_> {
    /// Tries one of the database's candidates (`slam_it.edge.loops._try_candidate`).
    ///
    /// # Arguments
    ///
    /// * `keyframe` - The candidate's id.
    /// * `score` - Its group's score.
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
    ) -> Result<(CandidateSummary, Option<Found>)> {
        let (m, options) = (self.m, self.options);
        let settings = &options.loops;
        let c = keyframe_row(m, keyframe)?;
        let mut group = vec![c];
        group.extend(best_links(m, c, options)?);
        let mut summary = CandidateSummary {
            keyframe,
            score,
            ..CandidateSummary::default()
        };
        if group.iter().any(|g| self.linked.contains(g)) {
            summary.near = true;
            return Ok((summary, None));
        }
        let merged = merged_by_word(m, self.db, self.k, &group, options)?;
        summary.matches = Some(matched_count(&merged.points));
        summary.best = Some(m.kf_id[merged.best]);
        if matched_count(&merged.points) < settings.min_matches {
            return Ok((summary, None));
        }
        let (model, inliers) =
            similarity_by_ransac(m, self.camera, self.k, &merged, self.fixed_scale, options)?;
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
            self.fixed_scale,
            options,
        )?
        else {
            return Ok((summary, None));
        };
        let confirmed = confirmations(m, self.camera, self.k, merged.best, &similarity, options)?;
        summary.confirmations = Some(confirmed);
        let found = Found {
            best: merged.best,
            similarity,
            points: points.iter().map(|&p| m.pt_id[p]).collect(),
            reprojected: matched_count(&fine),
            matched_points: fine,
            confirmations: confirmed,
        };
        Ok((summary, Some(found)))
    }

    /// Tries the database's candidates (`slam_it.edge.loops._search`).
    ///
    /// # Returns
    ///
    /// The best that passed, and how the search went.
    ///
    /// # Errors
    ///
    /// The database's or a candidate's.
    fn search(&self) -> Result<(Option<Found>, DetectionSummary)> {
        let candidates =
            database::loop_candidates(self.db, self.m, self.m.kf_id[self.k], self.options)?;
        let mut tried = Vec::new();
        let mut best: Option<Found> = None;
        for (&keyframe, &score) in candidates.keyframes.iter().zip(&candidates.scores) {
            let (summary, found) = self.try_candidate(keyframe, score)?;
            tried.push(summary);
            if let Some(found) = found
                && best
                    .as_ref()
                    .is_none_or(|best| found.reprojected > best.reprojected)
            {
                best = Some(found);
            }
        }
        let summary = DetectionSummary {
            sharing: candidates.sharing,
            scored: candidates.scored,
            candidates: tried,
            ..DetectionSummary::of(Outcome::None)
        };
        Ok((best, summary))
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
    let fixed_scale = mode != Mode::Monocular;
    let settings = &options.loops;
    let held = pending.map_or(0, |p| p.confirmations);
    if m.keyframes() < settings.min_keyframes {
        return Ok(Detection {
            pending: pending.cloned(),
            closed: None,
            summary: DetectionSummary {
                confirmations: held,
                ..DetectionSummary::of(Outcome::Skipped)
            },
        });
    }
    let mut confirmation = None;
    let mut dropped = false;
    let mut carried_on = None;
    if let Some(pending) = pending {
        let (carried, confirmed) = carry(m, camera, k, pending, fixed_scale, options)?;
        if confirmed.confirmed
            && let Some(carried) = carried
        {
            let summary = DetectionSummary {
                confirmation: Some(confirmed),
                ..DetectionSummary::of(Outcome::Confirmed)
            };
            return Ok(settle(carried, summary, options));
        }
        dropped = carried.is_none();
        carried_on = carried;
        confirmation = Some(confirmed);
    }
    let searching = Searching {
        m,
        db,
        camera,
        k,
        linked: linked(m, k, options)?,
        fixed_scale,
        options,
    };
    let (found, mut searched) = searching.search()?;
    searched.confirmation = confirmation;
    let Some(found) = found else {
        searched.outcome = if dropped {
            Outcome::Dropped
        } else {
            Outcome::None
        };
        searched.confirmations = carried_on.as_ref().map_or(0, |p| p.confirmations);
        return Ok(Detection {
            pending: carried_on,
            closed: None,
            summary: searched,
        });
    };
    let started = Pending {
        matched: m.kf_id[found.best],
        last: keyframe,
        similarity: found.similarity,
        confirmations: found.confirmations,
        misses: 0,
        points: found.points,
        matched_points: found.matched_points,
    };
    searched.outcome = Outcome::Started;
    Ok(settle(started, searched, options))
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
        let (best, distance) = nearest(m, p, &near);
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
/// ORB-SLAM's `OptimizeEssentialGraph`): unit information, the matched keyframe fixed,
/// `loops.essential_iterations` of Levenberg-Marquardt from `loops.essential_damping`, on Sim(3)
/// but in stereo and RGB-D.
///
/// # Arguments
///
/// * `m` - The map.
/// * `start` - Each keyframe's `S_iw` the graph starts from, by row.
/// * `before` - Each one's pose before the correction, by row.
/// * `loop_links` - Each moved keyframe's (row) links new across the loop.
/// * `ends` - The current and matched keyframes' rows.
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
pub fn essential_graph(
    m: &Map,
    start: &[Matrix4<f64>],
    before: &[Matrix4<f64>],
    loop_links: &BTreeMap<usize, BTreeSet<usize>>,
    ends: (usize, usize),
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
        &[ends.1],
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
    let adjusted = graphs::bundle_adjust(
        &poses,
        &m.pt_xyz,
        &graphs::Observations {
            cameras: &cameras,
            point_ids: &point_ids,
            pixels: &pixels,
            right: &right,
            information: &information,
        },
        &graphs::Lens {
            model: camera.model,
            intrinsics: camera.intrinsics.clone(),
            bf: camera.focal() * camera.baseline,
        },
        &fixed,
        &graphs::Options {
            max_iterations: options.loops.global_iterations,
            ..graphs::Options::default()
        },
    )?;
    m.kf_pose = adjusted.poses.iter().map(sim3::inverse).collect();
    m.pt_xyz = adjusted.points;
    let all: Vec<usize> = (0..m.points()).collect();
    map::refresh_points(m, &all)?;
    Ok(adjusted.summary)
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
    let moved_ids: Vec<i64> = moved_rows.iter().map(|&r| m.kf_id[r]).collect();
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
    let mut fused = 0;
    for (&keyframe, &old) in moved_ids.iter().zip(&moved_rows) {
        let r = keyframe_row(m, keyframe)?;
        fused += fuse_by_similarity(m, camera, r, &corrected[&old], &closed.points, options)?;
    }
    let mut moved_now = Vec::with_capacity(moved_ids.len());
    for &keyframe in &moved_ids {
        moved_now.push(keyframe_row(m, keyframe)?);
    }
    let now: BTreeSet<usize> = moved_now.iter().copied().collect();
    let mut loop_links = BTreeMap::new();
    for (&r, old) in moved_now.iter().zip(&moved_rows) {
        let new: BTreeSet<usize> = linked(m, r, options)?
            .difference(&previous[old])
            .copied()
            .filter(|n| !now.contains(n))
            .collect();
        loop_links.insert(r, new);
    }
    let mut start = m.kf_pose.clone();
    for (&r, old) in moved_now.iter().zip(&moved_rows) {
        start[r] = corrected[old];
    }
    // The keyframes are the same rows before and after: nothing here adds or removes one.
    let current = keyframe_row(m, closed.current)?;
    let essential = essential_graph(
        m,
        &start,
        &before,
        &loop_links,
        (current, matched),
        fixed_scale,
        options,
    )?;
    move_by_graph(m, &start, &essential.poses, &moved_by)?;
    map::add_loop_edge(m, closed.matched, closed.current)?;
    let global_ba = if options.loops.global_ba {
        Some(global_bundle_adjust(m, camera, options)?)
    } else {
        None
    };
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
