//! Native tests for the covisibility graph (`slam_it_edge::covisibility`), against the
//! brute-force fixture `tests/fixtures/edge/covisibility.json` that the Python and C++ tests
//! share: each keyframe's links and connected keyframes, tracking's local map, the parent a
//! keyframe adopts, the parents a culled keyframe's children get, a keyframe culled, and the
//! strong links.

use nalgebra::{Matrix4, Vector3};
use serde_json::Value;
use slam_it_edge::covisibility::{
    Link, adopt, connected, cull, links, local_map, reparent, strong_links,
};
use slam_it_edge::map::{Map, add_loop_edge, check_map, erase_keyframe, index_observations};
use slam_it_edge::mapping::{fuse_targets, local_ba_keyframes};
use slam_it_edge::options::{Mode, Options};
use slam_it_foundations::error::ErrorKind;

/// The fixture, read once per test.
fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/edge/covisibility.json"
    ))
    .expect("the fixture is valid JSON")
}

/// A JSON array of integers.
fn integers(value: &Value) -> Vec<i64> {
    value
        .as_array()
        .expect("an array")
        .iter()
        .map(|v| v.as_i64().expect("an integer"))
        .collect()
}

/// A JSON array of rows.
fn rows(value: &Value) -> Vec<usize> {
    integers(value)
        .into_iter()
        .map(|v| usize::try_from(v).expect("a row"))
        .collect()
}

/// Asserts links found are the fixture's rows and weights, in order.
fn assert_links(found: &[Link], expected: &Value, what: &str) {
    assert_eq!(
        found.iter().map(|l| l.row).collect::<Vec<_>>(),
        rows(&expected[0]),
        "{what}: rows"
    );
    assert_eq!(
        found.iter().map(|l| l.weight).collect::<Vec<_>>(),
        rows(&expected[1]),
        "{what}: weights"
    );
}

/// A case's map: the columns the graph reads (ids, parents, each keypoint's point), the rest
/// blank, as the Python test builds it, so that it can be culled.
fn map_of(keyframes: &Value) -> Map {
    let mut m = Map::empty(8, 1.2);
    for k in keyframes.as_array().expect("keyframes") {
        m.kf_id.push(k["id"].as_i64().expect("an id"));
        m.kf_parent.push(k["parent"].as_i64().expect("a parent"));
        m.kf_time.push(0.0);
        m.kf_pose.push(Matrix4::identity());
        for p in integers(&k["points"]) {
            m.kp_point.push(p);
            m.kp_xy.push([0.0, 0.0]);
            m.kp_level.push(0);
            m.kp_angle.push(0);
            m.kp_descriptor.push([0; 32]);
            m.kp_bearing.push(Vector3::z());
            m.kp_depth.push(f64::NAN);
            m.kp_right.push(f64::NAN);
        }
        m.kf_start.push(m.kp_point.len());
    }
    let mut points: Vec<i64> = m.kp_point.iter().copied().filter(|&p| p >= 0).collect();
    points.sort_unstable();
    points.dedup();
    for _ in &points {
        m.pt_xyz.push(Vector3::z()); // in front of every camera, so normals stay finite
        m.pt_descriptor.push([0; 32]);
        m.pt_normal.push(Vector3::z());
        m.pt_min_distance.push(1.0);
        m.pt_max_distance.push(1.0);
        m.pt_visible.push(1);
        m.pt_found.push(1);
        m.pt_first_kf.push(0);
        m.pt_reference_kf.push(0);
    }
    m.next_kf_id = m.kf_id.iter().max().map_or(0, |&id| id + 1);
    m.next_pt_id = points.last().map_or(0, |&p| p + 1);
    m.pt_id = points;
    index_observations(&mut m);
    m
}

/// A vector without one element.
fn without(mut values: Vec<i64>, k: usize) -> Vec<i64> {
    values.remove(k);
    values
}

/// Culling a case's keyframe: refused for the map's first, else its children given the
/// fixture's new parents and it erased; erasing it unculled refused if it has any.
fn check_cull(case: &Value, m: &Map, k: usize, options: &Options) {
    let name = case["name"].as_str().expect("a name");
    let mut culled = m.clone();
    if m.kf_id[k] == 0 {
        assert_eq!(
            cull(&mut culled, k, options, &[]).unwrap_err().kind,
            ErrorKind::InvalidInput,
            "{name}: the map's first keyframe is not culled"
        );
        return;
    }
    cull(&mut culled, k, options, &[]).unwrap();
    assert_eq!(
        culled.kf_id,
        without(m.kf_id.clone(), k),
        "{name}: row {k} culled"
    );
    assert_eq!(
        culled.kf_parent,
        without(integers(&case["reparented"][k]), k),
        "{name}: row {k}'s children reparented first"
    );
    check_map(&culled).unwrap();
    assert!(
        culled
            .pt_normal
            .iter()
            .all(|n| n.iter().all(|x| x.is_finite())),
        "{name}: row {k} culled, its points' normals finite"
    );
    if m.kf_parent.contains(&m.kf_id[k]) {
        let mut erased = m.clone();
        assert_eq!(
            erase_keyframe(&mut erased, k).unwrap_err().kind,
            ErrorKind::InvalidInput,
            "{name}: row {k}, a parent, is not erased"
        );
    }
    let mut held = m.clone();
    cull(&mut held, k, options, &[m.kf_id[k]]).unwrap();
    assert_eq!(held.kf_id, m.kf_id, "{name}: row {k}, held, is spared");
    let mut looped = m.clone();
    let other = m.kf_id[usize::from(k == 0)];
    add_loop_edge(&mut looped, m.kf_id[k], other).unwrap();
    cull(&mut looped, k, options, &[]).unwrap();
    assert_eq!(
        looped.kf_id, m.kf_id,
        "{name}: row {k}, with a loop edge, is spared"
    );
}

/// A case's settings.
fn options_of(settings: &Value) -> Options {
    let count = |name: &str| usize::try_from(settings[name].as_i64().expect(name)).expect(name);
    let mut out = Options::default();
    out.covisibility.min_points = count("min_points");
    out.covisibility.local_keyframes = count("local_keyframes");
    out.covisibility.local_neighbours = count("local_neighbours");
    out.covisibility.strong_points = count("strong_points");
    out
}

#[test]
fn every_case_matches_the_brute_force() {
    for case in fixture()["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("a name");
        let m = map_of(&case["keyframes"]);
        let options = options_of(&case["settings"]);
        for k in 0..m.keyframes() {
            assert_links(
                &links(&m, k, &options).unwrap(),
                &case["links"][k],
                &format!("{name}: links of row {k}"),
            );
            assert_links(
                &connected(&m, k).unwrap(),
                &case["connected"][k],
                &format!("{name}: the keyframes connected to row {k}"),
            );
            let mut orphaned = m.clone();
            let id = m.kf_id[k];
            for (r, parent) in orphaned.kf_parent.iter_mut().enumerate() {
                if r == k || *parent == id {
                    *parent = -1; // its children too, as the Python test leaves them
                }
            }
            adopt(&mut orphaned, k).unwrap();
            assert_eq!(
                orphaned.kf_parent[k],
                case["adopted"][k].as_i64().unwrap(),
                "{name}: the parent row {k} adopts"
            );
            let mut culled = m.clone();
            reparent(&mut culled, k, &options).unwrap();
            assert_eq!(
                culled.kf_parent,
                integers(&case["reparented"][k]),
                "{name}: the parents after culling row {k}"
            );
            check_cull(case, &m, k, &options);
        }
        for local in case["local_maps"].as_array().expect("local maps") {
            let found = local_map(&m, &integers(&local["points"]), &options).unwrap();
            assert_eq!(
                found.keyframes,
                rows(&local["keyframes"]),
                "{name}: a local map"
            );
            let reference = local["reference"].as_i64().unwrap();
            assert_eq!(
                found.reference,
                usize::try_from(reference).ok(),
                "{name}: its reference"
            );
        }
        let count =
            |name: &str| usize::try_from(case["fusion"][name].as_i64().expect(name)).expect(name);
        let mut fusion = options;
        fusion.mapping.fuse_neighbours = count("fuse_neighbours");
        fusion.mapping.mono_fuse_neighbours = count("mono_fuse_neighbours");
        fusion.mapping.fuse_second_neighbours = count("fuse_second_neighbours");
        for (mode, key) in [
            (Mode::Monocular, "monocular"),
            (Mode::Rgbd, "other"),
            (Mode::Stereo, "other"),
        ] {
            assert_eq!(
                fuse_targets(&m, mode, &fusion).unwrap(),
                integers(&case["fuse_targets"][key]),
                "{name}: fusion's targets, {key}"
            );
        }
        assert_eq!(
            local_ba_keyframes(&m, &fusion).unwrap(),
            rows(&case["local_ba"]),
            "{name}: local BA's keyframes"
        );
        let strong: Vec<[i64; 3]> = case["strong_links"]
            .as_array()
            .expect("strong links")
            .iter()
            .map(|l| {
                let v = integers(l);
                [v[0], v[1], v[2]]
            })
            .collect();
        assert_eq!(
            strong_links(&m, &options),
            strong,
            "{name}: the strong links"
        );
    }
}

#[test]
fn a_bad_row_or_point_is_refused() {
    let m = map_of(&fixture()["cases"][0]["keyframes"]);
    let options = Options::default();
    assert!(links(&m, m.keyframes(), &options).is_err());
    assert!(connected(&m, m.keyframes()).is_err());
    let mut past = m.clone();
    assert_eq!(
        cull(&mut past, m.keyframes(), &options, &[])
            .unwrap_err()
            .kind,
        ErrorKind::InvalidInput
    );
    let mut none = Map::empty(8, 1.2);
    assert_eq!(
        cull(&mut none, 0, &options, &[]).unwrap_err().kind,
        ErrorKind::InvalidInput
    );
    assert!(local_map(&m, &[3, -2], &options).is_err());
}
