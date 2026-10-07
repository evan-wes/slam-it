//! Local mapping, growing and cleaning the map around each new keyframe: the Rust core's
//! mirror of `slamlab.edge.mapping` (ORB-SLAM's `LocalMapping`, lite), whose docstring states
//! the steps.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{Matrix3, Matrix4, Vector2, Vector3};
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::matching::{self, FeatureGrid};
use slamlab_foundations::orb::Keypoint;
use slamlab_foundations::triangulation;

use crate::adjust::bundle_adjust;
use crate::frames::{Camera, Frame, Tracked, check_camera};
use crate::map::{self, Map};
use crate::options::{Mode, Options, check_options};
use crate::residuals::{chi2, sigma2, threshold};

/// Moves a camera-frame point into the world: `R^T (X - t)`.
///
/// # Arguments
///
/// * `pose` - The camera's `T_cam_world`.
/// * `x` - The point in the camera's frame.
///
/// # Returns
///
/// The point in the world.
fn to_world(pose: &Matrix4<f64>, x: &Vector3<f64>) -> Vector3<f64> {
    pose.fixed_view::<3, 3>(0, 0).transpose() * (x - pose.fixed_view::<3, 1>(0, 3))
}

/// A keyframe's keypoint rows.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// Its rows.
fn rows_of(m: &Map, k: usize) -> std::ops::Range<usize> {
    m.kf_start[k]..m.kf_start[k + 1]
}

/// The world point a keypoint's depth gives (`UnprojectStereo`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - Its keyframe's row.
/// * `row` - The keypoint's row.
///
/// # Returns
///
/// The point.
fn from_depth(m: &Map, k: usize, row: usize) -> Vector3<f64> {
    let b = m.kp_bearing[row];
    to_world(&m.kf_pose[k], &(b * (m.kp_depth[row] / b.z)))
}

/// Checks the camera and the settings, and that the map has a keyframe.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check_newest(m: &Map, camera: &Camera, options: &Options) -> Result<()> {
    check_camera(camera)?;
    check_options(options)?;
    if m.keyframes() == 0 {
        return fail(
            ErrorKind::InvalidInput,
            "m: expected a map with at least one keyframe",
        );
    }
    Ok(())
}

/// The depth below which a point is close (`slamlab.edge.mapping.close_depth`).
///
/// # Arguments
///
/// * `camera` - The camera (its baseline).
/// * `options` - The factor.
///
/// # Returns
///
/// `close_depth` baselines, in metres.
#[must_use]
pub fn close_depth(camera: &Camera, options: &Options) -> f64 {
    options.close_depth * camera.baseline
}

/// Decides whether a tracked frame becomes a keyframe (`slamlab.edge.mapping.need_keyframe`,
/// ORB-SLAM's `NeedNewKeyFrame`).
///
/// # Arguments
///
/// * `m` - The map, with at least one keyframe.
/// * `camera` - The camera.
/// * `depth` - The frame's keypoints' depths (NaN where unknown).
/// * `tracked` - Its pose and matches.
/// * `mode` - The pipeline's mode.
/// * `options` - The ratios and counts.
///
/// # Returns
///
/// Whether to insert a keyframe.
///
/// # Errors
///
/// `InvalidInput` for a map without keyframes, matches that are not one per depth, or an
/// invalid camera or settings.
#[allow(
    clippy::cast_precision_loss,
    reason = "counts of keypoints, exact in f64"
)]
pub fn need_keyframe(
    m: &Map,
    camera: &Camera,
    depth: &[f64],
    tracked: &Tracked,
    mode: Mode,
    options: &Options,
) -> Result<bool> {
    check_newest(m, camera, options)?;
    if tracked.points.len() != depth.len() {
        return fail(
            ErrorKind::InvalidInput,
            "tracked.points: expected one point id (or -1) per depth",
        );
    }
    // NeedNewKeyFrame's nKFs <= 2.
    let min_observations = i64::try_from(if m.keyframes() <= 2 {
        options.ref_min_observations_early
    } else {
        options.ref_min_observations
    })
    .unwrap_or(i64::MAX);
    let rows: Vec<usize> = rows_of(m, m.keyframes() - 1)
        .filter_map(|r| m.point_row(m.kp_point[r]))
        .collect();
    let tracked_reference = map::observation_weight(m, &rows)?
        .iter()
        .filter(|&&w| w >= min_observations)
        .count();
    let mut need_close = false;
    if mode != Mode::Monocular {
        let limit = close_depth(camera, options);
        let (mut tracked_close, mut untracked_close) = (0, 0);
        for (i, &z) in depth.iter().enumerate() {
            if z > 0.0 && z < limit {
                if tracked.points[i] >= 0 {
                    tracked_close += 1;
                } else {
                    untracked_close += 1;
                }
            }
        }
        need_close =
            tracked_close < options.close_tracked && untracked_close > options.close_untracked;
    }
    let mut ratio = if m.keyframes() < 2 {
        options.first_ref_ratio
    } else {
        options.ref_ratio
    };
    if mode == Mode::Monocular {
        ratio = options.mono_ref_ratio;
    }
    let count = tracked.inliers();
    let weak = (count as f64) < tracked_reference as f64 * ratio || need_close;
    Ok(weak && count > options.min_keyframe_inliers)
}

/// Creates points from the newest keyframe's close depth
/// (`slamlab.edge.mapping.create_points_from_depth`, ORB-SLAM's `CreateNewKeyFrame`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `camera` - The camera.
/// * `options` - The counts and close depth.
///
/// # Errors
///
/// `InvalidInput` for a map without keyframes, or an invalid camera or settings.
pub fn create_points_from_depth(m: &mut Map, camera: &Camera, options: &Options) -> Result<()> {
    check_newest(m, camera, options)?;
    let k = m.keyframes() - 1;
    let mut order: Vec<usize> = rows_of(m, k)
        .filter(|&r| m.kp_depth[r] > 0.0 && m.kp_bearing[r].z > 0.0)
        .collect();
    order.sort_by(|&a, &b| m.kp_depth[a].total_cmp(&m.kp_depth[b]));
    let limit = close_depth(camera, options);
    let mut xyz = Vec::new();
    let mut observers = Vec::new();
    for (count, &row) in (1..).zip(&order) {
        if m.kp_point[row] < 0 {
            xyz.push(from_depth(m, k, row));
            observers.push(vec![row]);
        }
        if m.kp_depth[row] > limit && count > options.min_new_points {
            break;
        }
    }
    if xyz.is_empty() {
        return Ok(());
    }
    map::add_points(m, &xyz, &observers)
}

/// Culls recently created points tracking rarely finds (`slamlab.edge.mapping.cull_points`,
/// ORB-SLAM's `MapPointCulling`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `mode` - The pipeline's mode.
/// * `options` - The ratio and minimums.
///
/// # Errors
///
/// `InvalidInput` for a map without keyframes, or invalid settings.
#[allow(
    clippy::cast_precision_loss,
    reason = "visible and found counts, exact in f64"
)]
pub fn cull_points(m: &mut Map, mode: Mode, options: &Options) -> Result<()> {
    check_options(options)?;
    let Some(&current) = m.kf_id.last() else {
        return fail(
            ErrorKind::InvalidInput,
            "m: expected a map with at least one keyframe",
        );
    };
    let recent_keyframes = i64::try_from(options.recent_point_keyframes).unwrap_or(i64::MAX);
    let recent: Vec<usize> = (0..m.points())
        .filter(|&p| current - m.pt_first_kf[p] <= recent_keyframes)
        .collect();
    if recent.is_empty() {
        return Ok(());
    }
    let minimum = i64::try_from(if mode == Mode::Monocular {
        options.min_point_observations
    } else {
        options.min_point_observations_depth
    })
    .unwrap_or(i64::MAX);
    let weights = map::observation_weight(m, &recent)?;
    let mut erase = vec![false; m.points()];
    for (&p, &weight) in recent.iter().zip(&weights) {
        let ratio = m.pt_found[p] as f64 / m.pt_visible[p] as f64;
        let age = current - m.pt_first_kf[p];
        erase[p] = ratio < options.found_ratio || (age >= 2 && weight <= minimum);
    }
    map::erase_points(m, &erase)
}

/// A keyframe's keypoints observing no point, with a bearing in front.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// Their rows.
fn free_rows(m: &Map, k: usize) -> Vec<usize> {
    rows_of(m, k)
        .filter(|&r| m.kp_point[r] < 0 && m.kp_bearing[r].z > 0.0)
        .collect()
}

/// The relative pose `T_2_1` of two keyframes: `R2 R1^T` and `t2 - R t1`.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k1` - The first keyframe's row.
/// * `k2` - The second's.
///
/// # Returns
///
/// The rotation and translation.
fn relative(m: &Map, k1: usize, k2: usize) -> (Matrix3<f64>, Vector3<f64>) {
    let (pose1, pose2) = (&m.kf_pose[k1], &m.kf_pose[k2]);
    let rotation: Matrix3<f64> =
        pose2.fixed_view::<3, 3>(0, 0) * pose1.fixed_view::<3, 3>(0, 0).transpose();
    let t = pose2.fixed_view::<3, 1>(0, 3) - rotation * pose1.fixed_view::<3, 1>(0, 3);
    (rotation, t)
}

/// A bearing on the normalised plane `z = 1`.
///
/// # Arguments
///
/// * `b` - The bearing.
///
/// # Returns
///
/// `b / b_z` (its third coordinate exactly 1).
fn normalised(b: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(b.x / b.z, b.y / b.z, 1.0)
}

/// The two keyframes' epipolar geometry, as the triangulation matching reads it.
struct Epipolar {
    /// The essential matrix `[t]x R` of `T_2_1`.
    essential: Matrix3<f64>,
    /// The epipole on keyframe 2's normalised plane, if camera 1's centre is not on its focal
    /// plane.
    epipole: Option<Vector2<f64>>,
    /// The focal length, in pixels.
    focal: f64,
    /// The band's chi-square bound (`epipolar_chi2`).
    chi2: f64,
    /// The least squared distance from the epipole at level 0 (`min_epipole_distance2`).
    epipole_distance2: f64,
}

/// Whether a candidate pair passes the epipolar band and the epipole test.
///
/// # Arguments
///
/// * `m` - The map.
/// * `epipolar` - The two keyframes' geometry.
/// * `line` - Keypoint 1's epipolar line in keyframe 2.
/// * `row1` - Keypoint 1's row.
/// * `row2` - Keypoint 2's row.
///
/// # Returns
///
/// Whether it passes.
#[allow(clippy::cast_precision_loss, reason = "levels are small")]
fn epipolar_ok(
    m: &Map,
    epipolar: &Epipolar,
    line: &Vector3<f64>,
    row1: usize,
    row2: usize,
) -> bool {
    let x2 = normalised(&m.kp_bearing[row2]);
    let numerator = line.dot(&x2);
    let distance2 = (numerator * numerator) / (line.x * line.x + line.y * line.y)
        * (epipolar.focal * epipolar.focal);
    if !(distance2 < epipolar.chi2 * sigma2(m, m.kp_level[row2])) {
        return false;
    }
    if let Some(epipole) = epipolar.epipole
        && m.kp_depth[row1].is_nan()
        && m.kp_depth[row2].is_nan()
    {
        let gap = Vector2::new(x2.x, x2.y) - epipole;
        let near = (gap.x * gap.x + gap.y * gap.y) * (epipolar.focal * epipolar.focal);
        if near < epipolar.epipole_distance2 * m.scale_factor.powf(m.kp_level[row2] as f64) {
            return false;
        }
    }
    true
}

/// Matches free keypoints for triangulation, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k1` - The first keyframe's row.
/// * `k2` - The second's.
/// * `options` - The epipolar band and the epipole's distance.
///
/// # Returns
///
/// The pairs.
fn triangulation_pairs(
    m: &Map,
    camera: &Camera,
    k1: usize,
    k2: usize,
    options: &Options,
) -> Vec<[usize; 2]> {
    let rows1 = free_rows(m, k1);
    let rows2 = free_rows(m, k2);
    let (rotation, t) = relative(m, k1, k2);
    let skew = Matrix3::new(0.0, -t.z, t.y, t.z, 0.0, -t.x, -t.y, t.x, 0.0);
    let epipolar = Epipolar {
        essential: skew * rotation,
        epipole: (t.z != 0.0).then(|| Vector2::new(t.x / t.z, t.y / t.z)),
        focal: camera.focal(),
        chi2: options.epipolar_chi2,
        epipole_distance2: options.min_epipole_distance2,
    };
    // For each keyframe-1 keypoint, the nearest candidate (ties: the first); then one match per
    // keyframe-2 keypoint, the nearest keeping it (ties: the lower keyframe-1 keypoint).
    let mut claims: BTreeMap<usize, (i64, usize)> = BTreeMap::new();
    for (i, &r1) in rows1.iter().enumerate() {
        let line = epipolar.essential * normalised(&m.kp_bearing[r1]);
        let mut best: Option<(i64, usize)> = None;
        for (j, &r2) in rows2.iter().enumerate() {
            let d = matching::distance(&m.kp_descriptor[r1], &m.kp_descriptor[r2]);
            if d <= options.th_low
                && best.is_none_or(|(b, _)| d < b)
                && epipolar_ok(m, &epipolar, &line, r1, r2)
            {
                best = Some((d, j));
            }
        }
        if let Some((d, j)) = best
            && claims.get(&j).is_none_or(|&(held, _)| d < held)
        {
            claims.insert(j, (d, i));
        }
    }
    let mut out: Vec<[usize; 2]> = claims
        .into_iter()
        .map(|(j, (_, i))| [rows1[i], rows2[j]])
        .collect();
    out.sort_unstable();
    out
}

/// Matches two keyframes' free keypoints for triangulation
/// (`slamlab.edge.mapping.match_for_triangulation`, ORB-SLAM's `SearchForTriangulation`,
/// lite).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k1` - The first keyframe's row.
/// * `k2` - The second's.
/// * `options` - The epipolar band and the epipole's distance.
///
/// # Returns
///
/// Keypoint rows `(in k1, in k2)`, by `k1` keypoint.
///
/// # Errors
///
/// `InvalidInput` for a row out of range, the same row twice, or an invalid camera or
/// settings.
pub fn match_for_triangulation(
    m: &Map,
    camera: &Camera,
    k1: usize,
    k2: usize,
    options: &Options,
) -> Result<Vec<[usize; 2]>> {
    check_camera(camera)?;
    check_options(options)?;
    if k1 >= m.keyframes() || k2 >= m.keyframes() || k1 == k2 {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "k1, k2: expected two different keyframe rows in [0, {})",
                m.keyframes()
            ),
        );
    }
    Ok(triangulation_pairs(m, camera, k1, k2, options))
}

/// How a pair is triangulated (`CreateNewMapPoints`' choice).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    /// Not at all: no depth and too little parallax.
    None,
    /// From the two rays.
    Rays,
    /// From keypoint 1's depth.
    Depth1,
    /// From keypoint 2's depth.
    Depth2,
}

/// The parallax cosine a keypoint's depth gives: `cos(2 atan2(b / 2, z))`.
///
/// # Arguments
///
/// * `camera` - The camera.
/// * `depth` - The depth.
///
/// # Returns
///
/// The cosine.
fn stereo_cos(camera: &Camera, depth: f64) -> f64 {
    (2.0 * (camera.baseline / 2.0).atan2(depth)).cos()
}

/// Chooses how to triangulate a pair: from the rays if their parallax beats what depth gives
/// (ORB-SLAM takes the depth parallax of the first keypoint with depth only).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `ks` - The two keyframes' rows.
/// * `pair` - The keypoint rows.
/// * `options` - The parallax limit.
///
/// # Returns
///
/// The source.
fn choose_source(
    m: &Map,
    camera: &Camera,
    ks: [usize; 2],
    pair: [usize; 2],
    options: &Options,
) -> Source {
    let [r1, r2] = pair;
    let ray1 = m.kf_pose[ks[0]].fixed_view::<3, 3>(0, 0).transpose() * m.kp_bearing[r1];
    let ray2 = m.kf_pose[ks[1]].fixed_view::<3, 3>(0, 0).transpose() * m.kp_bearing[r2];
    let cos_rays = ray1.dot(&ray2);
    let has1 = !m.kp_depth[r1].is_nan();
    let has2 = !m.kp_depth[r2].is_nan();
    let cos1 = if has1 {
        stereo_cos(camera, m.kp_depth[r1])
    } else {
        cos_rays + 1.0
    };
    let cos2 = if !has1 && has2 {
        stereo_cos(camera, m.kp_depth[r2])
    } else {
        cos_rays + 1.0
    };
    if cos_rays < cos1.min(cos2)
        && cos_rays > 0.0
        && (has1 || has2 || cos_rays < options.max_parallax_cos)
    {
        Source::Rays
    } else if has1 && cos1 < cos2 {
        Source::Depth1
    } else if has2 && cos2 < cos1 {
        Source::Depth2
    } else {
        Source::None
    }
}

/// Whether a point passes the chi-square test in a keyframe.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `row` - The keypoint's row.
/// * `x` - The point.
/// * `options` - The thresholds.
///
/// # Returns
///
/// Whether it passes.
///
/// # Errors
///
/// The test's error.
fn reprojects(
    m: &Map,
    camera: &Camera,
    k: usize,
    row: usize,
    x: &Vector3<f64>,
    options: &Options,
) -> Result<bool> {
    let error = chi2(
        camera,
        &[map::to_camera(&m.kf_pose[k], x)],
        &[m.kp_xy[row]],
        &[m.kp_right[row]],
        &[sigma2(m, m.kp_level[row])],
    )?;
    Ok(error[0] < threshold(m.kp_right[row], options))
}

/// Whether a triangulated point's distances agree with its keypoints' levels.
///
/// # Arguments
///
/// * `m` - The map.
/// * `ks` - The two keyframes' rows.
/// * `pair` - The keypoint rows.
/// * `x` - The point.
/// * `options` - The scale ratio.
///
/// # Returns
///
/// Whether they agree.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    reason = "levels are small"
)]
fn scale_consistent(
    m: &Map,
    ks: [usize; 2],
    pair: [usize; 2],
    x: &Vector3<f64>,
    options: &Options,
) -> bool {
    let distance1 = (x - m.centre(ks[0])).norm();
    let distance2 = (x - m.centre(ks[1])).norm();
    if !(distance1 > 0.0 && distance2 > 0.0) {
        return false;
    }
    let ratio_distance = distance2 / distance1;
    let ratio_octave = m
        .scale_factor
        .powf((m.kp_level[pair[0]] as i64 - m.kp_level[pair[1]] as i64) as f64);
    let factor = options.scale_ratio * m.scale_factor;
    !(ratio_distance * factor < ratio_octave || ratio_distance > ratio_octave * factor)
}

/// Triangulated points and their observers.
type Triangulated = (Vec<Vector3<f64>>, Vec<Vec<usize>>);

/// Triangulates matched pairs and keeps those passing every check
/// (`slamlab.edge.mapping._triangulate`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `ks` - The two keyframes' rows.
/// * `pairs` - The keypoint rows.
/// * `options` - The thresholds.
///
/// # Returns
///
/// The kept points and their pairs' observers.
///
/// # Errors
///
/// The triangulation's or the test's error.
fn triangulate(
    m: &Map,
    camera: &Camera,
    ks: [usize; 2],
    pairs: &[[usize; 2]],
    options: &Options,
) -> Result<Triangulated> {
    let sources: Vec<Source> = pairs
        .iter()
        .map(|&pair| choose_source(m, camera, ks, pair, options))
        .collect();
    let by_rays: Vec<usize> = (0..pairs.len())
        .filter(|&i| sources[i] == Source::Rays)
        .collect();
    let mut xyz = vec![Vector3::repeat(f64::NAN); pairs.len()];
    if !by_rays.is_empty() {
        let bearings1: Vec<Vector3<f64>> =
            by_rays.iter().map(|&i| m.kp_bearing[pairs[i][0]]).collect();
        let bearings2: Vec<Vector3<f64>> =
            by_rays.iter().map(|&i| m.kp_bearing[pairs[i][1]]).collect();
        let (rotation, t) = relative(m, ks[0], ks[1]);
        let mut pose = Matrix4::identity();
        pose.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
        pose.fixed_view_mut::<3, 1>(0, 3).copy_from(&t);
        let found = triangulation::two_view(&pose, &bearings1, &bearings2)?;
        for (&i, point) in by_rays.iter().zip(&found.points) {
            xyz[i] = to_world(&m.kf_pose[ks[0]], point);
        }
    }
    let mut out: Triangulated = (Vec::new(), Vec::new());
    for (i, &pair) in pairs.iter().enumerate() {
        match sources[i] {
            Source::Depth1 => xyz[i] = from_depth(m, ks[0], pair[0]),
            Source::Depth2 => xyz[i] = from_depth(m, ks[1], pair[1]),
            Source::Rays | Source::None => {}
        }
        if xyz[i].x.is_nan() {
            continue;
        }
        let in1 = reprojects(m, camera, ks[0], pair[0], &xyz[i], options)?;
        let in2 = reprojects(m, camera, ks[1], pair[1], &xyz[i], options)?;
        if in1 && in2 && scale_consistent(m, ks, pair, &xyz[i], options) {
            out.0.push(xyz[i]);
            out.1.push(vec![pair[0], pair[1]]);
        }
    }
    Ok(out)
}

/// A keyframe's median scene depth, the lower middle (`ComputeSceneMedianDepth(2)`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// The depth, or `None` if it sees no point.
fn median_depth(m: &Map, k: usize) -> Option<f64> {
    let mut depths: Vec<f64> = rows_of(m, k)
        .filter_map(|r| m.point_row(m.kp_point[r]))
        .map(|p| map::to_camera(&m.kf_pose[k], &m.pt_xyz[p]).z)
        .collect();
    if depths.is_empty() {
        return None;
    }
    depths.sort_by(f64::total_cmp);
    Some(depths[(depths.len() - 1) / 2])
}

/// Triangulates the newest keyframe's free keypoints against the window's
/// (`slamlab.edge.mapping.triangulate_new_points`, ORB-SLAM's `CreateNewMapPoints`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `camera` - The camera.
/// * `mode` - The pipeline's mode.
/// * `options` - The thresholds.
///
/// # Errors
///
/// `InvalidInput` for a map without keyframes, or an invalid camera or settings.
pub fn triangulate_new_points(
    m: &mut Map,
    camera: &Camera,
    mode: Mode,
    options: &Options,
) -> Result<()> {
    check_newest(m, camera, options)?;
    let k1 = m.keyframes() - 1;
    let mut others = m.window(options.window);
    others.pop();
    others.reverse();
    others.truncate(if mode == Mode::Monocular {
        options.mono_triangulation_neighbours
    } else {
        options.triangulation_neighbours
    });
    for k2 in others {
        let baseline = (m.centre(k2) - m.centre(k1)).norm();
        if mode == Mode::Monocular {
            match median_depth(m, k2) {
                Some(median) if baseline / median >= options.min_baseline_depth => {}
                _ => continue,
            }
        } else if baseline < camera.baseline {
            continue;
        }
        let pairs = triangulation_pairs(m, camera, k1, k2, options);
        if pairs.is_empty() {
            continue;
        }
        let (xyz, observers) = triangulate(m, camera, [k1, k2], &pairs, options)?;
        if !xyz.is_empty() {
            map::add_points(m, &xyz, &observers)?;
        }
    }
    Ok(())
}

/// Replaces one point by another (`slamlab.edge.mapping._merge_points`, `MapPoint::Replace`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `lose` - The lost point's id.
/// * `keep` - The kept point's id.
///
/// # Errors
///
/// The map's error.
fn merge_points(m: &mut Map, lose: i64, keep: i64) -> Result<()> {
    let (Some(p_lose), Some(p_keep)) = (m.point_row(lose), m.point_row(keep)) else {
        return fail(
            ErrorKind::InvalidInput,
            "merge: both points must be in the map",
        );
    };
    let kept_in: BTreeSet<usize> = m.pt_observations[p_keep]
        .iter()
        .map(|&r| m.keyframe_of(r))
        .collect();
    for r in m.pt_observations[p_lose].clone() {
        let to = if kept_in.contains(&m.keyframe_of(r)) {
            -1
        } else {
            keep
        };
        map::observe(m, r, to)?;
    }
    m.pt_visible[p_keep] += m.pt_visible[p_lose];
    m.pt_found[p_keep] += m.pt_found[p_lose];
    let mut erase = vec![false; m.points()];
    erase[p_lose] = true;
    map::erase_points(m, &erase)?;
    let kept: Vec<usize> = m.point_row(keep).into_iter().collect();
    map::refresh_points(m, &kept)
}

/// A keyframe's keypoints as the grid reads them.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// Their positions and levels.
fn keypoints_of(m: &Map, k: usize) -> Vec<Keypoint> {
    rows_of(m, k)
        .map(|r| Keypoint {
            xy: m.kp_xy[r],
            level: m.kp_level[r],
            level_xy: [0, 0],
            response: 0,
            angle: m.kp_angle[r],
        })
        .collect()
}

/// The keypoint row a point fuses into in a keyframe, if any
/// (`slamlab.edge.mapping._fuse_target`, `Fuse`' search).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `grid` - Its keypoints' grid.
/// * `p` - The point's row.
/// * `options` - The window and thresholds.
///
/// # Returns
///
/// The row, or `None`.
///
/// # Errors
///
/// The level prediction's, the grid's or the test's error.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    reason = "levels are small"
)]
fn fuse_target(
    m: &Map,
    camera: &Camera,
    k: usize,
    grid: &FeatureGrid,
    p: usize,
    options: &Options,
) -> Result<Option<usize>> {
    let at = map::to_camera(&m.kf_pose[k], &m.pt_xyz[p]);
    if !(at.z > 0.0) {
        return Ok(None);
    }
    let pixel = camera.project(&at);
    let offset = m.pt_xyz[p] - m.centre(k);
    let distance = offset.norm();
    if !(camera.inside(&pixel)
        && map::in_distance_range(m, p, distance, options)
        && offset.dot(&m.pt_normal[p]) >= options.view_cos_limit * distance)
    {
        return Ok(None);
    }
    let level = map::predict_level(m, &[p], &[distance])?[0];
    let near = grid.in_area(
        [pixel.x, pixel.y],
        options.fuse_window * m.scale_factor.powf(level as f64),
        level as i64 - 1,
        level as i64,
    )?;
    let mut best: Option<(i64, usize)> = None;
    for j in near {
        let row = m.kf_start[k] + j;
        let d = matching::distance(&m.pt_descriptor[p], &m.kp_descriptor[row]);
        if reprojects(m, camera, k, row, &m.pt_xyz[p], options)? && best.is_none_or(|(b, _)| d < b)
        {
            best = Some((d, row));
        }
    }
    Ok(best.and_then(|(d, row)| (d <= options.th_low).then_some(row)))
}

/// Fuses one point into a keyframe (`slamlab.edge.mapping._fuse_one`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `grid` - Its keypoints' grid.
/// * `id` - The point's id.
/// * `options` - The window and thresholds.
///
/// # Errors
///
/// The search's or the map's error.
fn fuse_one(
    m: &mut Map,
    camera: &Camera,
    k: usize,
    grid: &FeatureGrid,
    id: i64,
    options: &Options,
) -> Result<()> {
    let Some(p) = m.point_row(id) else {
        return Ok(()); // merged away
    };
    if m.pt_observations[p].iter().any(|&r| m.keyframe_of(r) == k) {
        return Ok(()); // already in the keyframe
    }
    let Some(row) = fuse_target(m, camera, k, grid, p, options)? else {
        return Ok(());
    };
    let held = m.kp_point[row];
    let Some(q) = m.point_row(held) else {
        return map::observe(m, row, id);
    };
    let weights = map::observation_weight(m, &[q, p])?;
    if weights[0] > weights[1] {
        merge_points(m, id, held)
    } else {
        merge_points(m, held, id)
    }
}

/// Fuses points into a keyframe, by id.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `k` - The keyframe's row.
/// * `ids` - The points' ids.
/// * `options` - The window and thresholds.
///
/// # Errors
///
/// The grid's, the search's or the map's error.
fn fuse_into(m: &mut Map, camera: &Camera, k: usize, ids: &[i64], options: &Options) -> Result<()> {
    let grid = FeatureGrid::build(&keypoints_of(m, k))?;
    for &id in ids {
        fuse_one(m, camera, k, &grid, id, options)?;
    }
    Ok(())
}

/// Fuses the newest keyframe's points with the window's (`slamlab.edge.mapping.fuse_points`,
/// ORB-SLAM's `SearchInNeighbors`, lite).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `camera` - The camera.
/// * `options` - The window and thresholds.
///
/// # Errors
///
/// `InvalidInput` for a map without keyframes, or an invalid camera or settings.
pub fn fuse_points(m: &mut Map, camera: &Camera, options: &Options) -> Result<()> {
    check_newest(m, camera, options)?;
    let mut targets: Vec<i64> = m
        .window(options.window)
        .iter()
        .map(|&k| m.kf_id[k])
        .collect();
    targets.pop();
    let newest = m.keyframes() - 1;
    let own: Vec<i64> = m
        .observed_by(&[newest])
        .iter()
        .map(|&p| m.pt_id[p])
        .collect();
    for &target in &targets {
        if let Some(k) = m.keyframe_row(target) {
            fuse_into(m, camera, k, &own, options)?;
        }
    }
    let rows: Vec<usize> = targets
        .iter()
        .filter_map(|&id| m.keyframe_row(id))
        .collect();
    let theirs: Vec<i64> = m.observed_by(&rows).iter().map(|&p| m.pt_id[p]).collect();
    let newest = m.keyframes() - 1;
    fuse_into(m, camera, newest, &theirs, options)?;
    let refreshed = m.observed_by(&[newest]);
    map::refresh_points(m, &refreshed)
}

/// Counts a keyframe's points that enough other keyframes see at its level or finer
/// (`slamlab.edge.mapping._redundant`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `rows` - The keypoint rows counted.
/// * `threshold` - The observers needed.
///
/// # Returns
///
/// The count.
///
/// # Errors
///
/// The map's error.
fn redundant(m: &Map, k: usize, rows: &[usize], threshold: usize) -> Result<usize> {
    let point_rows: Vec<usize> = rows
        .iter()
        .filter_map(|&r| m.point_row(m.kp_point[r]))
        .collect();
    let weights = map::observation_weight(m, &point_rows)?;
    let limit = i64::try_from(threshold).unwrap_or(i64::MAX);
    Ok(rows
        .iter()
        .zip(&point_rows)
        .zip(&weights)
        .filter(|&((&row, &p), &weight)| {
            weight > limit
                && m.pt_observations[p]
                    .iter()
                    .filter(|&&r| m.keyframe_of(r) != k && m.kp_level[r] <= m.kp_level[row] + 1)
                    .count()
                    >= threshold
        })
        .count())
}

/// Culls the window's redundant keyframes (`slamlab.edge.mapping.cull_keyframes`, ORB-SLAM's
/// `KeyFrameCulling`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `camera` - The camera.
/// * `mode` - The pipeline's mode.
/// * `options` - The fraction and count.
///
/// # Errors
///
/// `InvalidInput` for an invalid camera or settings.
#[allow(
    clippy::cast_precision_loss,
    reason = "counts of keypoints, exact in f64"
)]
pub fn cull_keyframes(m: &mut Map, camera: &Camera, mode: Mode, options: &Options) -> Result<()> {
    check_camera(camera)?;
    check_options(options)?;
    let limit = close_depth(camera, options);
    let mut ids: Vec<i64> = m
        .window(options.window)
        .iter()
        .map(|&k| m.kf_id[k])
        .collect();
    ids.pop();
    for id in ids {
        if id == 0 {
            continue;
        }
        let Some(k) = m.keyframe_row(id) else {
            continue;
        };
        let rows: Vec<usize> = rows_of(m, k)
            .filter(|&r| {
                m.kp_point[r] >= 0
                    && (mode == Mode::Monocular || (m.kp_depth[r] >= 0.0 && m.kp_depth[r] <= limit))
            })
            .collect();
        if rows.is_empty() {
            continue;
        }
        let count = redundant(m, k, &rows, options.redundant_observers)?;
        if count as f64 > options.redundant_fraction * rows.len() as f64 {
            map::erase_keyframe(m, k)?;
        }
    }
    Ok(())
}

/// Inserts a keyframe and runs local mapping on it (`slamlab.edge.mapping.process`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `camera` - The camera.
/// * `frame` - The new keyframe's frame.
/// * `tracked` - Its pose and matches.
/// * `mode` - The pipeline's mode.
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` for matches that are not one per keypoint, or an invalid camera or
/// settings.
pub fn process(
    m: &mut Map,
    camera: &Camera,
    frame: &Frame,
    tracked: &Tracked,
    mode: Mode,
    options: &Options,
) -> Result<()> {
    map::insert_keyframe(m, frame, &tracked.pose, &tracked.points)?;
    if mode != Mode::Monocular {
        create_points_from_depth(m, camera, options)?;
    }
    cull_points(m, mode, options)?;
    triangulate_new_points(m, camera, mode, options)?;
    fuse_points(m, camera, options)?;
    if m.keyframes() >= options.ba_min_keyframes {
        let fixed: Vec<bool> = m.kf_id.iter().map(|&id| id == 0).collect();
        let window = m.window(options.window);
        bundle_adjust(m, camera, &window, &fixed, options)?;
    }
    cull_keyframes(m, camera, mode, options)
}
