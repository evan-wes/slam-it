//! The covisibility graph: the Rust core's mirror of `slam_it.edge.covisibility`.
//!
//! Two keyframes' weight is the number of points both observe, counted from the map's
//! observations each time it is asked for. A keyframe is linked to those it shares at least
//! `covisibility.min_points` with, or, if it shares that many with none, to the one it shares
//! most with; its links run most shared first, ties broken by the lower id. Only the spanning
//! tree is stored ([`Map::kf_parent`]). The Python module's docstring gives the rules and their
//! ORB-SLAM sources.

use slam_it_foundations::error::{ErrorKind, Result, check_range, fail};

use crate::map::{self, Map};
use crate::options::Options;

/// A link: another keyframe's row, and the points the two share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Link {
    /// The other keyframe's row.
    pub row: usize,
    /// The points both observe.
    pub weight: usize,
}

/// Tracking's local map (`slam_it.edge.covisibility.LocalMap`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LocalMap {
    /// Its keyframes' rows: K1, the most shared first, then K2 in the order added.
    pub keyframes: Vec<usize>,
    /// The reference keyframe's row (K1's first), or `None`.
    pub reference: Option<usize>,
}

/// The keyframes that observe a point, each once, by row ascending.
fn observers(m: &Map, p: usize) -> Vec<usize> {
    let mut out: Vec<usize> = m.pt_observations[p]
        .iter()
        .map(|&row| m.keyframe_of(row))
        .collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// The rows sharing a point, most shared first, ties by the lower id.
fn ranked(m: &Map, shared: &[usize]) -> Vec<usize> {
    let mut out: Vec<usize> = (0..shared.len()).filter(|&r| shared[r] > 0).collect();
    out.sort_by(|&a, &b| {
        shared[b]
            .cmp(&shared[a])
            .then_with(|| m.kf_id[a].cmp(&m.kf_id[b]))
    });
    out
}

/// Checks a keyframe row.
fn check_row(m: &Map, k: usize) -> Result<()> {
    if m.keyframes() == 0 {
        return fail(ErrorKind::InvalidInput, "k: the map has no keyframes");
    }
    check_range("k", k, 0, m.keyframes() - 1)
}

/// Adds a keyframe's row to the local map.
fn add(keyframes: &mut Vec<usize>, inside: &mut [bool], row: usize) {
    keyframes.push(row);
    inside[row] = true;
}

/// The rows whose parent is a keyframe, ascending.
fn children_of(m: &Map, k: usize) -> Vec<usize> {
    (0..m.keyframes())
        .filter(|&r| m.kf_parent[r] == m.kf_id[k])
        .collect()
}

/// The points a keyframe shares with each keyframe.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row, in range.
///
/// # Returns
///
/// One count per keyframe row, 0 for the keyframe itself.
#[must_use]
pub fn shared_points(m: &Map, k: usize) -> Vec<usize> {
    let mut out = vec![0; m.keyframes()];
    let mut points: Vec<usize> = m.kp_point[m.kf_start[k]..m.kf_start[k + 1]]
        .iter()
        .filter_map(|&id| m.point_row(id))
        .collect();
    points.sort_unstable();
    points.dedup();
    for p in points {
        for other in observers(m, p) {
            if other != k {
                out[other] += 1;
            }
        }
    }
    out
}

/// A keyframe's links from its shared points: those sharing at least `min_points`, or the one
/// sharing the most.
///
/// # Arguments
///
/// * `m` - The map.
/// * `shared` - Its shared points ([`shared_points`]).
/// * `min_points` - The links' least weight.
///
/// # Returns
///
/// The links, most shared first, ties by the lower id.
#[must_use]
pub fn links_of(m: &Map, shared: &[usize], min_points: usize) -> Vec<Link> {
    let order = ranked(m, shared);
    let mut out: Vec<Link> = order
        .iter()
        .filter(|&&r| shared[r] >= min_points)
        .map(|&row| Link {
            row,
            weight: shared[row],
        })
        .collect();
    if out.is_empty()
        && let Some(&row) = order.first()
    {
        out.push(Link {
            row,
            weight: shared[row],
        });
    }
    out
}

/// A keyframe's links (`slam_it.edge.covisibility.links`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
/// * `options` - The settings: `covisibility.min_points`.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
pub fn links(m: &Map, k: usize, options: &Options) -> Result<Vec<Link>> {
    check_row(m, k)?;
    Ok(links_of(
        m,
        &shared_points(m, k),
        options.covisibility.min_points,
    ))
}

/// Tracking's local map, ORB-SLAM's K1 and K2 (`slam_it.edge.covisibility.local_map`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `points` - The point each of the frame's keypoints was matched to, -1 for none.
/// * `options` - The settings: the `covisibility` group's.
///
/// # Errors
///
/// `InvalidInput` for an id below -1.
pub fn local_map(m: &Map, points: &[i64], options: &Options) -> Result<LocalMap> {
    if points.iter().any(|&id| id < -1) {
        return fail(
            ErrorKind::InvalidInput,
            "points: expected point ids (-1 or more)",
        );
    }
    let mut matched: Vec<usize> = points.iter().filter_map(|&id| m.point_row(id)).collect();
    matched.sort_unstable();
    matched.dedup();
    let mut shared = vec![0; m.keyframes()];
    for p in matched {
        for k in observers(m, p) {
            shared[k] += 1;
        }
    }
    let k1 = ranked(m, &shared);
    let Some(&reference) = k1.first() else {
        return Ok(LocalMap::default());
    };
    let settings = &options.covisibility;
    let mut keyframes = k1.clone();
    let mut inside = vec![false; m.keyframes()];
    for &k in &k1 {
        inside[k] = true;
    }
    for &k in &k1 {
        if keyframes.len() > settings.local_keyframes {
            break;
        }
        let best = links_of(m, &shared_points(m, k), settings.min_points);
        // Each added before the next is looked for: a neighbour that is also a child counts once.
        if let Some(row) = best
            .iter()
            .take(settings.local_neighbours)
            .map(|link| link.row)
            .find(|&row| !inside[row])
        {
            add(&mut keyframes, &mut inside, row);
        }
        if let Some(row) = children_of(m, k).into_iter().find(|&row| !inside[row]) {
            add(&mut keyframes, &mut inside, row);
        }
        if let Some(row) = m.keyframe_row(m.kf_parent[k])
            && !inside[row]
        {
            add(&mut keyframes, &mut inside, row);
        }
    }
    Ok(LocalMap {
        keyframes,
        reference: Some(reference),
    })
}

/// Gives a keyframe its parent, if it has none yet (`slam_it.edge.covisibility.adopt`).
///
/// For the keyframe being inserted, the map's newest, every other already in the tree: given an
/// older keyframe without a parent, it may take a newer one whose own chain leads back to it, a
/// cycle.
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `k` - The keyframe's row.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
pub fn adopt(m: &mut Map, k: usize) -> Result<()> {
    check_row(m, k)?;
    if m.kf_id[k] == 0 || m.kf_parent[k] >= 0 {
        return Ok(());
    }
    if let Some(&best) = ranked(m, &shared_points(m, k)).first() {
        m.kf_parent[k] = m.kf_id[best];
    }
    Ok(())
}

/// Gives a keyframe's children new parents before it is culled
/// (`slam_it.edge.covisibility.reparent`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `k` - The keyframe's row.
/// * `options` - The settings: `covisibility.min_points`.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
pub fn reparent(m: &mut Map, k: usize, options: &Options) -> Result<()> {
    check_row(m, k)?;
    let mut children = children_of(m, k);
    if children.is_empty() {
        return Ok(());
    }
    // Each child's links, the culled keyframe's dropped first, as SetBadFlag erases them.
    let mut linked: Vec<Vec<Link>> = children
        .iter()
        .map(|&child| {
            let mut shared = shared_points(m, child);
            shared[k] = 0;
            links_of(m, &shared, options.covisibility.min_points)
        })
        .collect();
    let parent = m.kf_parent[k];
    let mut candidate = vec![false; m.keyframes()];
    if let Some(row) = m.keyframe_row(parent) {
        candidate[row] = true;
    }
    while !children.is_empty() {
        let mut best: Option<(usize, usize, usize)> = None; // (weight, child's index, row)
        for (i, links) in linked.iter().enumerate() {
            for link in links {
                if candidate[link.row] && best.is_none_or(|(most, _, _)| link.weight > most) {
                    best = Some((link.weight, i, link.row));
                }
            }
        }
        let Some((_, i, row)) = best else {
            break;
        };
        let child = children.remove(i);
        linked.remove(i);
        m.kf_parent[child] = m.kf_id[row];
        candidate[child] = true;
    }
    for child in children {
        m.kf_parent[child] = parent; // linked to no candidate: its own parent
    }
    Ok(())
}

/// Culls a keyframe: its children given new parents, then it erased (ORB-SLAM's `SetBadFlag`;
/// `slam_it.edge.covisibility.cull`). The one way a keyframe leaves the graph, as
/// [`map::erase_keyframe`] refuses a keyframe that is still a parent. The map's first keyframe
/// (id 0), its tree's root, is refused, as `SetBadFlag` leaves it. A keyframe with a loop edge,
/// or one of `held`, is spared, the map left as it was (`SetNotErase`).
///
/// # Arguments
///
/// * `m` - The map, changed in place.
/// * `k` - The keyframe's row.
/// * `options` - The settings: `covisibility.min_points`.
/// * `held` - The ids of the keyframes loop detection holds.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or the map's first keyframe.
pub fn cull(m: &mut Map, k: usize, options: &Options, held: &[i64]) -> Result<()> {
    check_row(m, k)?;
    if m.kf_id[k] == 0 {
        return fail(
            ErrorKind::InvalidInput,
            "k: keyframe 0 is the map's first, its tree's root, and is not culled",
        );
    }
    if map::has_loop(m, k) || held.contains(&m.kf_id[k]) {
        return Ok(()); // spared, as ORB-SLAM's SetNotErase spares it
    }
    reparent(m, k, options)?;
    map::erase_keyframe(m, k)
}

/// Every pair of keyframes sharing at least `covisibility.strong_points`
/// (`slam_it.edge.covisibility.strong_links`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `options` - The settings.
///
/// # Returns
///
/// Each pair's ids, the lower first, and the points they share, by the first id then the
/// second.
#[must_use]
pub fn strong_links(m: &Map, options: &Options) -> Vec<[i64; 3]> {
    let mut out = Vec::new();
    for a in 0..m.keyframes() {
        let shared = shared_points(m, a);
        for (b, &weight) in shared.iter().enumerate().skip(a + 1) {
            if weight >= options.covisibility.strong_points {
                out.push([
                    m.kf_id[a],
                    m.kf_id[b],
                    i64::try_from(weight).unwrap_or(i64::MAX),
                ]);
            }
        }
    }
    out
}
