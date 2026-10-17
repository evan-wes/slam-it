//! Native tests for the covisibility graph (`slam_it_edge::covisibility`), against the
//! brute-force fixture `tests/fixtures/edge/covisibility.json` that the Python and C++ tests
//! share: each keyframe's links, tracking's local map, the parent a keyframe adopts, the
//! parents a culled keyframe's children get, and the strong links.

use serde_json::Value;
use slam_it_edge::covisibility::{adopt, links, local_map, reparent, strong_links};
use slam_it_edge::map::{Map, index_observations};
use slam_it_edge::options::Options;

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

/// A case's map: the columns the graph reads (ids, parents, each keypoint's point).
fn map_of(keyframes: &Value) -> Map {
    let mut m = Map::empty(8, 1.2);
    for k in keyframes.as_array().expect("keyframes") {
        m.kf_id.push(k["id"].as_i64().expect("an id"));
        m.kf_parent.push(k["parent"].as_i64().expect("a parent"));
        m.kp_point.extend(integers(&k["points"]));
        m.kf_start.push(m.kp_point.len());
    }
    let mut points: Vec<i64> = m.kp_point.iter().copied().filter(|&p| p >= 0).collect();
    points.sort_unstable();
    points.dedup();
    m.pt_id = points;
    index_observations(&mut m);
    m
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
            let found = links(&m, k, &options).unwrap();
            let expected = &case["links"][k];
            assert_eq!(
                found.iter().map(|l| l.row).collect::<Vec<_>>(),
                rows(&expected[0]),
                "{name}: links of row {k}"
            );
            assert_eq!(
                found.iter().map(|l| l.weight).collect::<Vec<_>>(),
                rows(&expected[1]),
                "{name}: weights of row {k}"
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
    assert!(local_map(&m, &[3, -2], &options).is_err());
}
